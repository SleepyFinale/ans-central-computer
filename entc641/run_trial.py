#!/usr/bin/env python3
"""Run one run of the ENTC 641 experiment and collect everything it produces.

    python3 entc641/run_trial.py R03            # the run from data/plan.csv
    python3 entc641/run_trial.py R03 --redo     # repeat it; the old folder is kept, renamed
    python3 entc641/run_trial.py REF --reference --robots i   # reference map run
    python3 entc641/run_trial.py SHAKE1 --coded 1 1 -1 1      # a run outside the plan

For a plan run it:
  1. prints the settings and where to put the robots, and waits for Enter;
  2. starts the fleet (start_fleet.sh -ic --no-rviz) with the run's explorer
     parameters (EXPLORER_EXTRA_PARAMS) and top speed (FLEET_AFTER_SLAM_HOOK ->
     set_robot_speed.sh, applied before the robots join);
  3. records a central bag (maps, TF, goal status, battery) from just before the
     explorer starts, and each Pi's 1 Hz system monitor;
  4. takes t0 as the explorer's first goal and stops everything CAP_SEC (480 s)
     after it;
  5. writes data/runs/<id>/meta.json (settings, times, voltages, git commits,
     your note) next to the bag, the explorer event log, fleet logs and Pi CSVs.

A reference run uses the default settings, has no cap, and stops when you press
Enter (let one robot explore the whole area first). Ctrl+C stops any run
cleanly; it is then marked "aborted" and should be redone with --redo.
"""

import argparse
import csv
import glob
import json
import os
import re
import select
import shlex
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import design

WS = design.ROOT.parent
LETTERS = {'b': 'blinky', 'p': 'pinky', 'i': 'inky', 'c': 'clyde'}
SSH_OPTS = ['-o', 'StrictHostKeyChecking=no', '-o', 'ConnectTimeout=10']


def log(msg):
    print(f'[{datetime.now():%H:%M:%S}] {msg}', flush=True)


def ssh_target(robot):
    return os.environ.get(f'ENTC_SSH_{robot.upper()}', f'{robot}@{robot}')


def ssh(robot, command, timeout=30):
    return subprocess.run(stdin=subprocess.DEVNULL, args=['sshpass', '-e', 'ssh', *SSH_OPTS, ssh_target(robot), command],
                          capture_output=True, text=True, timeout=timeout)


def central_shell(command):
    """Run a command with the central ROS environment loaded."""
    return ['bash', '-c', f'set +u; source scripts/env/central_env.bash >/dev/null 2>&1; {command}']


def plan_row(run_id):
    if not design.PLAN_CSV.exists():
        sys.exit('no plan yet: run  python3 entc641/plan.py create')
    with open(design.PLAN_CSV, newline='') as f:
        for row in csv.DictReader(f):
            if row['run_id'] == run_id:
                return row
    sys.exit(f'{run_id} is not in {design.PLAN_CSV}')


def last_end_voltages():
    """End voltages of the most recent complete run, to warn before starting."""
    best = None
    for meta in design.RUNS.glob('*/meta.json'):
        try:
            m = json.loads(meta.read_text())
        except json.JSONDecodeError:
            continue
        if m.get('status') == 'complete' and m.get('ended_wall'):
            if best is None or m['ended_wall'] > best['ended_wall']:
                best = m
    return best


class Fleet:
    """start_fleet.sh in its own session, its output teed to fleet.out."""

    def __init__(self, robots, env, out_path, rviz):
        letters = ''.join(r[0] for r in robots)
        cmd = ['./scripts/core/start_fleet.sh', f'-{letters}']
        if not rviz:
            cmd.append('--no-rviz')
        self.out = open(out_path, 'w')
        self.proc = subprocess.Popen(stdin=subprocess.DEVNULL, args=cmd, cwd=WS, env=env, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True,
                                     start_new_session=True, bufsize=1)
        self.lines = []
        self.cond = threading.Condition()
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.proc.stdout:
            self.out.write(line)
            self.out.flush()
            s = line.rstrip()
            if s.startswith(('Ready:', '===', 'ERROR', 'WARNING', 'Fleet is up', 'Logs:')):
                print(f'    fleet| {s}', flush=True)
            with self.cond:
                self.lines.append(s)
                self.cond.notify_all()
        with self.cond:
            self.cond.notify_all()

    def wait_for(self, text, timeout):
        deadline = time.time() + timeout
        with self.cond:
            while True:
                if any(text in line for line in self.lines):
                    return True
                if self.proc.poll() is not None:
                    return False
                left = deadline - time.time()
                if left <= 0:
                    return False
                self.cond.wait(min(left, 1.0))

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        if self.alive():
            log('stopping the fleet (robot stages too; this takes up to a minute)')
            self.proc.send_signal(signal.SIGTERM)
            try:
                self.proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                log('fleet did not stop in 120 s; killing its process group')
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.out.close()


def start_bag(bag_dir, robots, out_path):
    topics = ['/map', '/tf', '/tf_static', '/explore/frontiers', '/map_merge/merge_state']
    for r in robots:
        topics += [f'/{r}/map', f'/{r}/plan', f'/{r}/battery_state',
                   f'/{r}/navigate_to_pose/_action/status']
    cmd = f'exec ros2 bag record --include-hidden-topics -o {shlex.quote(str(bag_dir))} ' + \
        ' '.join(topics)
    return subprocess.Popen(stdin=subprocess.DEVNULL, args=central_shell(cmd), cwd=WS, stdout=open(out_path, 'w'),
                            stderr=subprocess.STDOUT, start_new_session=True)


def stop_bag(proc):
    if proc is None or proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def live_voltage(robot):
    try:
        r = subprocess.run(stdin=subprocess.DEVNULL, args=central_shell(
            f'timeout 15 ros2 topic echo --once --field voltage /{robot}/battery_state'),
            cwd=WS, capture_output=True, text=True, timeout=25)
        return float(r.stdout.split()[0])
    except (subprocess.TimeoutExpired, ValueError, IndexError):
        return None


def monitor_tag(run_id, robot):
    return f'entc641-{run_id}-{robot}'


def start_pi_monitor(run_id, robot):
    tag = monitor_tag(run_id, robot)
    cmd = (f'cd ~/turtlebot3 || exit 1; setsid nohup ./scripts/monitor/pi_bottleneck_monitor.sh '
           f'--iface wlan0 --interval 1.0 --output /tmp/{tag}.csv >/tmp/{tag}.log 2>&1 '
           f'</dev/null & echo started')
    try:
        r = ssh(robot, cmd)
        ok = 'started' in r.stdout
    except subprocess.TimeoutExpired:
        ok = False
    log(f'{robot}: Pi monitor {"started" if ok else "FAILED to start (CPU data will be missing)"}')
    return ok


def stop_pi_monitor(run_id, robot, run_dir):
    tag = monitor_tag(run_id, robot)
    try:
        # The [e] keeps pkill from matching this SSH command's own shell.
        ssh(robot, f'pkill -INT -f "[e]{tag[1:]}.csv"; sleep 1; pkill -f "[e]{tag[1:]}.csv"; true')
        r = subprocess.run(stdin=subprocess.DEVNULL, args=['sshpass', '-e', 'scp', *SSH_OPTS,
                            f'{ssh_target(robot)}:/tmp/{tag}.csv', str(run_dir / f'pi_{robot}.csv')],
                           capture_output=True, text=True, timeout=60)
        if r.returncode != 0:
            log(f'{robot}: could not copy the Pi monitor CSV: {r.stderr.strip()}')
    except subprocess.TimeoutExpired:
        log(f'{robot}: Pi monitor stop/copy timed out')


def git_state(robot=None):
    cmd = 'git rev-parse --short HEAD && git branch --show-current && git status --porcelain | wc -l'
    try:
        if robot is None:
            r = subprocess.run(stdin=subprocess.DEVNULL, args=['bash', '-c', cmd], cwd=WS, capture_output=True, text=True, timeout=20)
        else:
            r = ssh(robot, f'cd ~/turtlebot3 && {cmd}')
        commit, branch, dirty = r.stdout.split()[:3]
        return {'commit': commit, 'branch': branch, 'uncommitted_files': int(dirty)}
    except (subprocess.TimeoutExpired, ValueError):
        return None


def read_events(run_dir):
    events = []
    for path in glob.glob(str(run_dir / 'session-*.jsonl')):
        with open(path) as f:
            for line in f:
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return events


# Only these mean a robot is home. "Return home abandoned" is not: the explorer
# gave up for now and retries later, so the robot may be anywhere.
HOME_MARKS = ('Return home succeeded', 'skipping NavigateToPose for return home')
STAMP = re.compile(r'\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\] \[multi_robot_explorer\]: (.*)')


def exploration_status(run_dir, robots):
    """(complete_t, home_t) in ROS seconds from the explorer's lines in central.log.

    complete_t: when it logged 'exploration complete!' (None if not yet).
    home_t: when the last robot reported home after that (None until all are).
    """
    complete_t = None
    home = {}
    for path in glob.glob(str(run_dir / 'fleet' / '*' / 'central.log')):
        with open(path, errors='replace') as f:
            for line in f:
                m = STAMP.search(line)
                if not m:
                    continue
                t, text = float(m.group(1)), m.group(2)
                if 'exploration complete!' in text:
                    if complete_t is None:
                        complete_t = t
                    continue
                if complete_t is None:
                    continue
                for r in robots:
                    if text.startswith(f'[{r}]') and any(k in text for k in HOME_MARKS):
                        home.setdefault(r, t)
    home_t = max(home.values()) if complete_t is not None and len(home) == len(robots) else None
    return complete_t, home_t


def goal_counts(events, t0_ns):
    out = {}
    for e in events:
        if (e.get('t_ros_ns') or 0) < t0_ns or not e.get('robot'):
            continue
        c = out.setdefault(e['robot'], {'sent': 0, 'SUCCEEDED': 0, 'ABORTED': 0})
        if e['event'] == 'goal_sent':
            c['sent'] += 1
        elif e['event'] == 'goal_result' and e.get('status_text') in c:
            c[e['status_text']] += 1
    return ', '.join(f"{r}: {c['sent']} sent {c['SUCCEEDED']} ok {c['ABORTED']} failed"
                     for r, c in sorted(out.items()))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_id')
    ap.add_argument('--reference', action='store_true',
                    help='reference map run: default settings, no cap, stop with Enter')
    ap.add_argument('--robots', default='ic', help='robot letters (default ic; reference: i)')
    ap.add_argument('--cap', type=float, default=design.CAP_SEC)
    ap.add_argument('--redo', action='store_true', help='repeat a run; keeps the old folder')
    ap.add_argument('--coded', type=float, nargs=4, metavar=('A', 'B', 'C', 'D'),
                    help='run these coded levels outside the plan (shakedown); not analyzed')
    ap.add_argument('--rviz', action='store_true', help='also open RViz')
    ap.add_argument('--yes', action='store_true', help='do not wait for Enter or ask for a note')
    ap.add_argument('--start-timeout', type=float, default=900,
                    help='seconds to wait for the fleet to come up (default 900)')
    args = ap.parse_args()

    run_id = args.run_id
    letters = args.robots
    if args.reference and letters == 'ic':
        letters = 'i'
    robots = [LETTERS[ch] for ch in letters]
    if args.reference:
        row = None
        settings = None
        cap = 0.0
    else:
        if args.coded:
            a, b, c, d = args.coded
            row = {'run_id': run_id, 'session': '', 'order': '', 'type': 'shakedown',
                   'condition': '', 'pair': '', 'arm': '', 'A': a, 'B': b, 'C': c, 'D': d}
        else:
            row = plan_row(run_id)
        settings = design.actual_settings(float(row['A']), float(row['B']),
                                          float(row['C']), float(row['D']))
        cap = args.cap

    run_dir = design.RUNS / run_id
    if run_dir.exists():
        if not args.redo:
            sys.exit(f'{run_dir} exists. Pass --redo to repeat {run_id} '
                     '(the old folder is kept, renamed).')
        old = run_dir.with_name(f'{run_id}.discarded-{datetime.now():%Y%m%d-%H%M%S}')
        run_dir.rename(old)
        log(f'kept the earlier attempt as {old.name}')
    run_dir.mkdir(parents=True)
    os.environ.setdefault('SSHPASS', os.environ.get('ROBOT_SSH_PASSWORD', 'ubuntu'))

    # ---- what to do before starting
    print()
    if args.reference:
        print(f'REFERENCE RUN {run_id}: {", ".join(robots)} with default settings, no cap.')
        print('Door open, as in every run. Let it explore the whole area, then press Enter.')
    else:
        print(f'RUN {run_id}  (session {row["session"]}, {row["type"]}'
              + (f', condition {row["condition"]}' if row['condition'] else '')
              + (f', pair {row["pair"]} {row["arm"]}' if row['pair'] else '') + ')')
        print(f'  A coordination_weight     {settings["coordination_weight"]}')
        print(f'  B start                   {settings["start"]}')
        print(f'  C potential : gain        {settings["potential_scale"]} : {settings["gain_scale"]}')
        print(f'  D max speed               {settings["max_speed"]} m/s')
        print(f'  cap                       {cap:.0f} s after the first goal')
        print(f'\nPlace the robots: {design.START_INSTRUCTIONS[settings["start"]]}')
        print('Door open. Batteries at or above '
              f'{design.MIN_START_VOLTAGE} V.')
        prev = last_end_voltages()
        if prev:
            for r, v in (prev.get('voltage_live_end') or {}).items():
                if v is not None:
                    flag = '  <- swap or charge this battery' if v < design.MIN_START_VOLTAGE else ''
                    print(f'  {r}: {v:.2f} V at the end of {prev["run_id"]}{flag}')
    if not args.yes:
        input('\nPress Enter to start (Ctrl+C to cancel) ')

    meta = {
        'run_id': run_id, 'reference': args.reference, 'robots': robots,
        'plan': row, 'settings': settings, 'cap_sec': cap,
        'started_wall': datetime.now().isoformat(timespec='seconds'),
        'git_central': git_state(),
        'git_robots': {r: git_state(r) for r in robots},
        'status': 'aborted',
    }

    def save_meta():
        (run_dir / 'meta.json').write_text(json.dumps(meta, indent=2) + '\n')

    save_meta()

    env = dict(os.environ)
    env.update({
        'FLEET_LOG_DIR': str(run_dir / 'fleet'),
        'CENTRAL_DEBUG_LOG_DIR': str(run_dir),
        'DEBUG_SESSION_ID': run_id,
        'CENTRAL_AUTO_KILL': 'true',
    })
    if settings:
        env['EXPLORER_EXTRA_PARAMS'] = (
            f'coordination_weight:={settings["coordination_weight"]} '
            f'potential_scale:={settings["potential_scale"]} '
            f'gain_scale:={settings["gain_scale"]}')
        env['FLEET_AFTER_SLAM_HOOK'] = str(design.ROOT / 'set_robot_speed.sh')
        env['ENTC_MAX_SPEED'] = str(settings['max_speed'])

    fleet = bag = None
    monitors = []
    try:
        log('starting the fleet')
        fleet = Fleet(robots, env, run_dir / 'fleet.out', args.rviz)
        if not fleet.wait_for('=== central ===', args.start_timeout):
            raise RuntimeError('the fleet did not reach the central stage (see fleet.out)')
        bag = start_bag(run_dir / 'bag', robots, run_dir / 'bag.out')
        log('bag recording')
        if not fleet.wait_for('Fleet is up', 300):
            raise RuntimeError('the central stack did not come up (see fleet.out)')
        for r in robots:
            if start_pi_monitor(run_id, r):
                monitors.append(r)
        meta['voltage_live_start'] = {}
        for r in robots:
            v = live_voltage(r)
            meta['voltage_live_start'][r] = v
            if v is None:
                log(f'{r}: battery voltage not readable yet')
            else:
                log(f'{r}: battery {v:.2f} V'
                    + ('   WARNING: below the start limit; consider Ctrl+C and a fresh battery'
                       if v < design.MIN_START_VOLTAGE else ''))
        save_meta()

        log('waiting for the explorer\'s first goal')
        t0_ns = None
        deadline = time.time() + 300
        while t0_ns is None:
            sent = [e['t_ros_ns'] for e in read_events(run_dir)
                    if e.get('event') == 'goal_sent' and e.get('t_ros_ns')]
            if sent:
                t0_ns = min(sent)
                break
            if not fleet.alive():
                raise RuntimeError('the fleet stopped before the first goal (see fleet.out)')
            if time.time() > deadline:
                raise RuntimeError('no goal from the explorer within 300 s')
            time.sleep(1.0)
        meta['t0_ros_ns'] = t0_ns
        meta['t0_wall'] = datetime.fromtimestamp(t0_ns * 1e-9).isoformat(timespec='seconds')
        save_meta()
        log(f't0 = first goal at {meta["t0_wall"]}')

        if cap > 0:
            end = t0_ns * 1e-9 + cap
            next_report = time.time() + 30
            meta['end_reason'] = 'cap'
            while time.time() < end:
                if not fleet.alive():
                    raise RuntimeError('the fleet stopped during the run (see fleet.out)')
                complete_t, home_t = exploration_status(run_dir, robots)
                if complete_t is not None and 't_complete_ros_ns' not in meta:
                    meta['t_complete_ros_ns'] = int(complete_t * 1e9)
                    save_meta()
                    log(f'exploration complete at {complete_t - t0_ns * 1e-9:.0f} s '
                        '(scoring ends here); waiting for the robots to get home')
                if home_t is not None:
                    meta['end_reason'] = 'explored_and_home'
                    meta['t_end_ros_ns'] = int(home_t * 1e9)
                    log(f'every robot home at {home_t - t0_ns * 1e-9:.0f} s; ending the run')
                    break
                if time.time() >= next_report:
                    el = time.time() - t0_ns * 1e-9
                    log(f'{el:4.0f}/{cap:.0f} s  {goal_counts(read_events(run_dir), t0_ns)}')
                    next_report += 30
                time.sleep(1.0)
            else:
                meta['t_end_ros_ns'] = int(end * 1e9)
                log('cap reached')
        else:
            # No cap: stop when exploration is complete and the robot is home,
            # or when you press Enter.
            print('Reference run: stops by itself once the area is mapped and the robot '
                  'is home; press Enter to stop sooner.', flush=True)
            meta['end_reason'] = 'enter'
            while True:
                if not fleet.alive():
                    raise RuntimeError('the fleet stopped during the run (see fleet.out)')
                complete_t, home_t = exploration_status(run_dir, robots)
                if home_t is not None:
                    meta['end_reason'] = 'explored_and_home'
                    meta['t_end_ros_ns'] = int(home_t * 1e9)
                    log('exploration complete and the robot is home')
                    break
                ready, _, _ = select.select([sys.stdin], [], [], 1.0)
                if ready:
                    sys.stdin.readline()
                    meta['t_end_ros_ns'] = int(time.time() * 1e9)
                    break
        meta['status'] = 'complete'
    except (KeyboardInterrupt, EOFError):
        log('interrupted: stopping and marking the run aborted')
        meta['status'] = 'aborted'
    except RuntimeError as e:
        log(f'ERROR: {e}')
        meta['status'] = 'failed'
        meta['error'] = str(e)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        meta['ended_wall'] = datetime.now().isoformat(timespec='seconds')
        if meta['status'] == 'complete':
            meta['voltage_live_end'] = {r: live_voltage(r) for r in robots}
        stop_bag(bag)
        for r in monitors:
            stop_pi_monitor(run_id, r, run_dir)
        if fleet is not None:
            fleet.stop()
        save_meta()

    signal.signal(signal.SIGINT, signal.default_int_handler)
    if meta['status'] == 'complete' and not args.yes:
        try:
            note = input('Note for this run (anything unusual; Enter for none): ').strip()
        except (EOFError, KeyboardInterrupt):
            note = ''
        if note:
            meta['note'] = note
            save_meta()
    log(f'{run_id}: {meta["status"]}. Files in {run_dir}')
    if meta['status'] != 'complete':
        log(f'repeat it with:  python3 entc641/run_trial.py {run_id} --redo')
        sys.exit(1)
    if not args.reference:
        log('next:  python3 entc641/plan.py status')


if __name__ == '__main__':
    main()
