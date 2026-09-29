#!/usr/bin/env python3
"""
Bridge Nav2 *action services* from central ROS_DOMAIN_ID to a robot domain.

domain_bridge YAML only forwards topics; action clients need send_goal /
get_result / cancel_goal *services* on the central domain. This relay forwards:

  - /<robot>/navigate_to_pose/_action/*
  - /<robot>/compute_path_to_pose/_action/*

(ComputePathToPose is required by multi_robot_explorer path precheck.)

Hidden action topics (status / feedback / result) still come from domain_bridge
YAML; this process only handles the three service types per action.

get_result stays open until Nav2 finishes the action. The robot worker therefore
keeps every service call in flight at once and pairs replies by request id.
Otherwise one slow ComputePathToPose get_result blocks send_goal and
cancel_goal, and the explorer times out forever without the robot ever moving.
"""

from __future__ import annotations

import argparse
import multiprocessing
import os
import queue
import signal
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from multiprocessing import Event, Queue

import rclpy
from action_msgs.srv import CancelGoal
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message


class RelayShuttingDown(Exception):
    """Raised when the relay is stopping; service callbacks should not log as hard errors."""


@dataclass(frozen=True)
class _ActionSpec:
    key: str
    base_path: str  # e.g. /pinky/navigate_to_pose (no /_action)
    send_goal_type: type
    get_result_type: type


def _action_specs(robot: str) -> list[_ActionSpec]:
    r = robot.strip('/')
    return [
        _ActionSpec(
            'nav',
            f'/{r}/navigate_to_pose',
            NavigateToPose.Impl.SendGoalService,
            NavigateToPose.Impl.GetResultService,
        ),
        _ActionSpec(
            'cp',
            f'/{r}/compute_path_to_pose',
            ComputePathToPose.Impl.SendGoalService,
            ComputePathToPose.Impl.GetResultService,
        ),
    ]


def _safe_destroy_node(node: Node | None) -> None:
    if node is None:
        return
    try:
        node.destroy_node()
    except Exception:
        pass


def _safe_rclpy_shutdown() -> None:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


def _safe_spin_once_worker(node: Node, timeout_sec: float = 0.0) -> bool:
    """Return False if the node should stop (invalid context / shutdown)."""
    try:
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    except Exception:
        if not rclpy.ok():
            return False
        raise
    return bool(rclpy.ok())


def _relay_log_error(node: Node | None, text: str) -> None:
    """Avoid rosout when context is tearing down (Ctrl+C on central)."""
    try:
        if node is not None and rclpy.ok():
            node.get_logger().error(text)
    except Exception:
        pass


def _robot_worker(
    q_in: Queue,
    q_out: Queue,
    ready_evt: Event,
    robot: str,
    robot_domain: int,
    wait_sec: float,
) -> None:
    """Owns robot-domain clients and executes forwarded action service calls."""
    os.environ['ROS_DOMAIN_ID'] = str(int(robot_domain))
    rclpy.init()
    node = rclpy.create_node(f'fleet_nav_action_relay_robot_{robot}')
    specs = _action_specs(robot)
    clients: dict[str, tuple] = {}

    try:
        for sp in specs:
            base = f'{sp.base_path}/_action'
            sg = node.create_client(sp.send_goal_type, f'{base}/send_goal')
            gr = node.create_client(sp.get_result_type, f'{base}/get_result')
            cg = node.create_client(CancelGoal, f'{base}/cancel_goal')
            clients[sp.key] = (sg, gr, cg)

        deadline = time.monotonic() + wait_sec
        while time.monotonic() < deadline and rclpy.ok():
            # Allow parent shutdown while still waiting for robot-side services.
            try:
                maybe_stop = q_in.get_nowait()
                if maybe_stop is None:
                    return
            except queue.Empty:
                pass
            all_ready = all(
                sg.service_is_ready() and gr.service_is_ready() and cg.service_is_ready()
                for sg, gr, cg in clients.values()
            )
            if all_ready:
                ready_evt.set()
                break
            time.sleep(0.2)
            if not _safe_spin_once_worker(node, 0.0):
                break

        if not ready_evt.is_set():
            if rclpy.ok():
                node.get_logger().error(
                    f'Timed out waiting for NavigateToPose + compute_path_to_pose '
                    f'services on robot domain {robot_domain} ({robot}).'
                )
            return

        node.get_logger().info(
            f'Nav2 action services reached on robot domain {robot_domain}; '
            'forwarding to central.'
        )

        # (req_id, future). get_result blocks until the action ends, so it must
        # not stall send_goal / cancel_goal behind spin_until_future_complete.
        inflight: list[tuple[str, object]] = []

        def _pump_inflight() -> bool:
            if not _safe_spin_once_worker(node, 0.0):
                return False
            still: list[tuple[str, object]] = []
            for req_id, fut in inflight:
                if not fut.done():
                    still.append((req_id, fut))
                    continue
                try:
                    rsp = fut.result()
                    if rsp is None:
                        q_out.put((req_id, 'err', 'service call returned no response'))
                    else:
                        q_out.put((req_id, 'ok', serialize_message(rsp)))
                except Exception as exc:  # noqa: BLE001
                    q_out.put((req_id, 'err', str(exc)))
            inflight[:] = still
            return True

        while rclpy.ok():
            try:
                item = q_in.get(timeout=0.05)
            except queue.Empty:
                if not _pump_inflight():
                    break
                continue
            if item is None:
                break
            req_id, action_key, kind, req_bytes = item
            sg, gr, cg = clients[action_key]
            client = {'sg': sg, 'gr': gr, 'cg': cg}.get(kind)
            if client is None:
                q_out.put((req_id, 'err', f'unknown relay kind: {kind!r}'))
                continue
            try:
                req = deserialize_message(req_bytes, client.srv_type.Request)
                inflight.append((req_id, client.call_async(req)))
            except Exception as exc:  # noqa: BLE001
                if not rclpy.ok():
                    break
                _relay_log_error(
                    node,
                    f'Relay worker error ({action_key}/{kind}): {exc}',
                )
                q_out.put((req_id, 'err', str(exc)))
            if not _pump_inflight():
                break
    finally:
        _safe_destroy_node(node)
        _safe_rclpy_shutdown()


class _RelayNode(Node):
    """Central-domain façade that exposes Nav2 action services to the explorer."""
    def __init__(
        self,
        robot: str,
        central_to_robot: Queue,
        robot_to_central: Queue,
    ):
        super().__init__(f'fleet_nav_action_relay_{robot}')
        self._c2r = central_to_robot
        self._r2c = robot_to_central
        self._shutdown = threading.Event()
        self._waiters: dict[str, queue.Queue] = {}
        self._waiters_lock = threading.Lock()
        self._demux_thread = threading.Thread(
            target=self._demux_responses, name=f'nav_relay_demux_{robot}', daemon=True)
        self._demux_thread.start()
        cb_group = ReentrantCallbackGroup()

        for sp in _action_specs(robot):
            base = f'{sp.base_path}/_action'
            self.create_service(
                sp.send_goal_type,
                f'{base}/send_goal',
                self._make_sg_handler(sp.key, sp.send_goal_type.Response),
                callback_group=cb_group,
            )
            self.create_service(
                sp.get_result_type,
                f'{base}/get_result',
                self._make_gr_handler(sp.key, sp.get_result_type.Response),
                callback_group=cb_group,
            )
            self.create_service(
                CancelGoal,
                f'{base}/cancel_goal',
                self._make_cg_handler(sp.key),
                callback_group=cb_group,
            )

    def _make_sg_handler(self, action_key: str, response_type: type):
        def handler(request, response):
            try:
                return self._forward(action_key, 'sg', request, response_type)
            except RelayShuttingDown:
                return response
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'[{action_key}] send_goal relay: {exc}')
                return response

        return handler

    def _make_gr_handler(self, action_key: str, response_type: type):
        def handler(request, response):
            try:
                return self._forward(action_key, 'gr', request, response_type)
            except RelayShuttingDown:
                return response
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'[{action_key}] get_result relay: {exc}')
                return response

        return handler

    def _make_cg_handler(self, action_key: str):
        def handler(request, response):
            try:
                return self._forward(action_key, 'cg', request, CancelGoal.Response)
            except RelayShuttingDown:
                return response
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'[{action_key}] cancel_goal relay: {exc}')
                return response

        return handler

    def _demux_responses(self) -> None:
        """Deliver robot-side replies to the matching _forward waiter."""
        while not self._shutdown.is_set():
            try:
                item = self._r2c.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                continue
            try:
                req_id, status, payload = item
            except Exception:
                continue
            with self._waiters_lock:
                waiter = self._waiters.get(req_id)
            if waiter is not None:
                waiter.put((status, payload))

    def _forward(self, action_key: str, kind: str, request, resp_type: type):
        if self._shutdown.is_set() or not rclpy.ok():
            raise RelayShuttingDown()
        req_id = uuid.uuid4().hex
        payload = serialize_message(request)
        waiter: queue.Queue = queue.Queue()
        with self._waiters_lock:
            self._waiters[req_id] = waiter
        try:
            self._c2r.put((req_id, action_key, kind, payload))
            deadline = time.monotonic() + 120.0
            while time.monotonic() < deadline:
                if self._shutdown.is_set() or not rclpy.ok():
                    raise RelayShuttingDown()
                try:
                    status, data = waiter.get(timeout=0.2)
                except queue.Empty:
                    continue
                if status != 'ok':
                    raise RuntimeError(str(data))
                return deserialize_message(data, resp_type)
            raise TimeoutError(f'Relay timeout for {action_key}/{kind}')
        finally:
            with self._waiters_lock:
                self._waiters.pop(req_id, None)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Relay Nav2 NavigateToPose + ComputePathToPose action services '
            'between central and robot ROS domains.'
        )
    )
    parser.add_argument('--robot', required=True, help='Namespace, e.g. pinky')
    parser.add_argument('--robot-domain', type=int, required=True)
    parser.add_argument('--central-domain', type=int, required=True)
    parser.add_argument(
        '--wait-for-robot-services-sec',
        type=float,
        default=120.0,
        help='Max time to wait for Nav2 action services on the robot domain.',
    )
    args = parser.parse_args()

    # Parent process stays on the central domain; child worker process switches
    # to the robot domain to issue real Nav2 service calls.
    os.environ['ROS_DOMAIN_ID'] = str(int(args.central_domain))

    ctx = multiprocessing.get_context('spawn')
    q_c2r: Queue = ctx.Queue()
    q_r2c: Queue = ctx.Queue()
    ready = ctx.Event()

    proc = ctx.Process(
        target=_robot_worker,
        args=(
            q_c2r,
            q_r2c,
            ready,
            args.robot,
            int(args.robot_domain),
            float(args.wait_for_robot_services_sec),
        ),
        daemon=False,
    )
    proc.start()

    if not ready.wait(timeout=args.wait_for_robot_services_sec + 5.0):
        print(
            'ERROR: robot-side relay did not become ready (Nav2 up on robot domain?).',
            file=sys.stderr,
        )
        q_c2r.put(None)
        proc.join(timeout=5.0)
        if proc.is_alive():
            proc.terminate()
            proc.join(timeout=2.0)
        q_c2r.close()
        q_r2c.close()
        q_c2r.join_thread()
        q_r2c.join_thread()
        return 1

    rclpy.init()
    node = _RelayNode(args.robot, q_c2r, q_r2c)
    # Service handlers block inside _forward. A thread pool lets get_result
    # stay open while send_goal and cancel_goal run.
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    # Do not raise KeyboardInterrupt from a signal handler: it can fire during
    # executor.shutdown() and corrupt teardown (rclpy guard_condition errors).
    shutdown_evt = threading.Event()

    def _request_shutdown(_signum=None, _frame=None):
        shutdown_evt.set()
        # Service handlers block inside spin_once. Set this immediately so
        # _forward stops within one queue poll instead of logging a 120s timeout.
        node._shutdown.set()

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    try:
        while rclpy.ok() and not shutdown_evt.is_set():
            try:
                executor.spin_once(timeout_sec=0.2)
            except ExternalShutdownException:
                break
            except Exception:
                if rclpy.ok() and not shutdown_evt.is_set():
                    raise
                break
    finally:
        # Unblock service threads waiting on _r2c before tearing down rclpy.
        node._shutdown.set()
        try:
            executor.shutdown(timeout_sec=5.0)
        except Exception:
            pass
        q_c2r.put(None)
        proc.join(timeout=10.0)
        if proc.is_alive():
            proc.terminate()
            proc.join(timeout=2.0)
        q_c2r.close()
        q_r2c.close()
        q_c2r.join_thread()
        q_r2c.join_thread()
        _safe_destroy_node(node)
        _safe_rclpy_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main() or 0)
