import tkinter as tk
from tkinter import ttk
import threading
import io
import subprocess
import os
import signal
from fabric import Connection

# --- CONFIGURATION ---
ROBOTS = {
    "blinky": {"host": "192.168.0.158", "user": "blinky", "type": "remote"},
    "pinky":  {"host": "192.168.0.194", "user": "pinky", "type": "remote"},
    "inky":   {"host": "192.168.0.139", "user": "inky", "type": "remote"},
    "clyde":  {"host": "192.168.0.236", "user": "clyde", "type": "remote"},
    # "Central": {"type": "local"},
    # "Rviz": {"type": "local"}
}

# Robot Commands
BRINGUP_CMD = (
    "source /opt/ros/humble/setup.bash && source ~/turtlebot3/install/setup.bash && "
    "source ~/turtlebot3/scripts/ros_domain_profile.bash && export TURTLEBOT3_MODEL=burger && "
    "export LDS_MODEL=LDS-02 && cd ~/turtlebot3 && source scripts/ros_robot_env.bash && "
    "ros2 launch turtlebot3_bringup robot.launch.py"
)

NAV_CMD = (
    "source /opt/ros/humble/setup.bash && source ~/turtlebot3/install/setup.bash && "
    "source ~/turtlebot3/scripts/ros_domain_profile.bash && export TURTLEBOT3_MODEL=burger && "
    "export LDS_MODEL=LDS-02 && cd ~/turtlebot3 && source scripts/ros_robot_env.bash && "
    "ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py "
    "use_sim_time:=false use_rviz:=false fleet_mode:=true nav2_use_local_slam_map:=true "
    "scan_costmap_max_hz:=6.0 enable_startup_map_seeding:=true"
)

# Central Commands (Local)
CENTRAL_1 = (
    "cd ~/ans-central-computer && source scripts/ros_domain_profile.bash && "
    "source scripts/ros_robot_env.bash && ./scripts/start_central.sh --comms-mode bridged_domains"
)
CENTRAL_2 = (
    "cd ~/ans-central-computer && source scripts/ros_domain_profile.bash && "
    "source scripts/ros_robot_env.bash && ./scripts/start_rviz_central.sh"
)

class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("TurtleBot3 & Central Controller")
        self.root.geometry("1000x650")

        self.btn_frame = ttk.Frame(root, padding=10)
        self.btn_frame.pack(side="top", fill="x")
        
        # Add a Kill All button for emergencies
        self.kill_all_btn = ttk.Button(self.btn_frame, text="☢ KILL ALL ROS", command=self.emergency_stop)
        self.kill_all_btn.pack(side="right", padx=10)

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(side="bottom", fill="both", expand=True, padx=10, pady=10)

        self.consoles, self.buttons, self.active_tasks = {}, {}, {}

        for name in ROBOTS:
            btn = ttk.Button(self.btn_frame, text=f"Start {name}", command=lambda n=name: self.toggle_task(n))
            btn.pack(side="left", padx=5)
            self.buttons[name] = btn

            tab = ttk.Frame(self.notebook)
            self.notebook.add(tab, text=name)
            txt = tk.Text(tab, bg="#1e1e1e", fg="#00ff00", font=("Courier", 9))
            txt.pack(fill="both", expand=True)
            self.consoles[name] = txt

    def log(self, name, msg):
        self.consoles[name].insert(tk.END, msg + "\n")
        self.consoles[name].see(tk.END)

    def emergency_stop(self):
        """Kills every known local ROS process as a backup."""
        subprocess.run("pkill -9 -f ros2", shell=True)
        subprocess.run("pkill -9 -f rviz2", shell=True)
        for name in list(self.active_tasks.keys()):
            self.toggle_task(name)

    def toggle_task(self, name):
        if name not in self.active_tasks:
            self.buttons[name].config(text=f"Stop {name}")
            threading.Thread(target=self.run_logic, args=(name,), daemon=True).start()
        else:
            self.log(name, ">> Stopping process group...")
            task = self.active_tasks[name]
            
            if isinstance(task, Connection):
                # Closing Fabric connection with pty=True kills remote processes
                task.close()
            elif isinstance(task, subprocess.Popen):
                try:
                    # KILL ENTIRE LOCAL PROCESS GROUP
                    os.killpg(os.getpgid(task.pid), signal.SIGTERM)
                except Exception as e:
                    self.log(name, f"!! Kill Error: {e}")
            
            if name in self.active_tasks: del self.active_tasks[name]
            self.buttons[name].config(text=f"Start {name}")

    def run_logic(self, name):
        config = ROBOTS[name]
        try:
            if config["type"] == "remote":
                conn = Connection(host=config["host"], user=config["user"], connect_kwargs={"password": "ubuntu"})
                self.active_tasks[name] = conn
                self.log(name, f">> SSH Connected to {name}.")
                script = f"({BRINGUP_CMD}) & sleep 20 && {NAV_CMD}"
                conn.run(script, pty=True, out_stream=self.ConsoleWriter(self, name))
            
            else:
                cmd = CENTRAL_1 if name == "Central" else CENTRAL_2
                self.log(name, f">> Launching Local {name}...")
                
                # preexec_fn=os.setsid creates a group ID for this process
                p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, 
                                     stderr=subprocess.STDOUT, text=True, 
                                     executable="/bin/bash", preexec_fn=os.setsid)
                self.active_tasks[name] = p

                for line in iter(p.stdout.readline, ""):
                    if name not in self.active_tasks: break
                    self.log(name, line.strip())

        except Exception as e:
            self.log(name, f"!! Error: {e}")
        finally:
            if name in self.active_tasks: del self.active_tasks[name]
            self.root.after(0, lambda: self.buttons[name].config(text=f"Start {name}"))

    class ConsoleWriter(io.TextIOBase):
        def __init__(self, app, name):
            self.app, self.name = app, name
        def write(self, s):
            self.app.root.after(0, lambda: self.app.log(self.name, s.strip()))
            return len(s)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotApp(root)
    root.mainloop()
