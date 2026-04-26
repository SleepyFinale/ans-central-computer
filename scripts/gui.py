import tkinter as tk
from tkinter import ttk
import threading
import io
from fabric import Connection

# --- ROBOT SETTINGS ---
ROBOTS = {
    "blinky": {"host": "192.168.0.158", "user": "blinky", "cmds": ["ls -la", "sleep 5"]},
    "pinky":  {"host": "192.168.0.194", "user": "pinky", "cmds": ["uptime", "sleep 5"]},
    "inky":   {"host": "192.168.0.139", "user": "inky", "cmds": ["df -h", "sleep 5"]},
    "clyde":  {"host": "192.168.0.236", "user": "clyde", "cmds": ["free -m", "sleep 5"]}
}


# Terminal 1: Bringup commands
BRINGUP_CMD = (
    "source /opt/ros/humble/setup.bash &&"
    "source ~/turtlebot3/install/setup.bash &&"
    "source ~/turtlebot3/scripts/ros_domain_profile.bash &&"
    "export TURTLEBOT3_MODEL=burger &&"
    "export LDS_MODEL=LDS-02 &&"
    "cd turtlebot3 &&"
    "export TURTLEBOT3_MODEL=burger && "
    "source scripts/ros_domain_profile.bash && "
    "source scripts/ros_robot_env.bash && "
    "ros2 launch turtlebot3_bringup robot.launch.py"
)

# Terminal 2: Navigation/SLAM commands
NAV_CMD = (
    "source /opt/ros/humble/setup.bash &&"
    "source ~/turtlebot3/install/setup.bash &&"
    "source ~/turtlebot3/scripts/ros_domain_profile.bash &&"
    "export TURTLEBOT3_MODEL=burger &&"
    "export LDS_MODEL=LDS-02 &&"
    "cd turtlebot3 &&"
    "export TURTLEBOT3_MODEL=burger && "
    "source scripts/ros_domain_profile.bash && "
    "source scripts/ros_robot_env.bash && "
    "ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py "
    "use_sim_time:=false use_rviz:=false fleet_mode:=true "
    "nav2_use_local_slam_map:=true scan_costmap_max_hz:=6.0 enable_startup_map_seeding:=true"
)

class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("TurtleBot3 Multi-Robot Controller")
        self.root.geometry("900x600")

        # UI Setup
        self.btn_frame = ttk.Frame(root, padding=10)
        self.btn_frame.pack(side="top", fill="x")
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(side="bottom", fill="both", expand=True, padx=10, pady=10)

        self.consoles = {}
        self.buttons = {}
        self.connections = {}  # Store active Fabric connections to close them on Stop

        for name in ROBOTS:
            # Buttons
            btn = ttk.Button(self.btn_frame, text=f"Start {name}", command=lambda n=name: self.toggle_robot(n))
            btn.pack(side="left", padx=5)
            self.buttons[name] = btn

            # Tabbed Consoles
            tab = ttk.Frame(self.notebook)
            self.notebook.add(tab, text=name)
            txt = tk.Text(tab, bg="#1e1e1e", fg="#00ff00", font=("Courier", 9))
            txt.pack(fill="both", expand=True)
            self.consoles[name] = txt

    def log(self, name, msg):
        self.consoles[name].insert(tk.END, msg + "\n")
        self.consoles[name].see(tk.END)

    def toggle_robot(self, name):
        if name not in self.connections:
            self.buttons[name].config(text=f"Stop {name}")
            threading.Thread(target=self.run_ros_stack, args=(name,), daemon=True).start()
        else:
            self.log(name, ">> Stopping robot...")
            # Closing the connection object usually kills the pty-hosted process
            self.connections[name].close()
            del self.connections[name]
            self.buttons[name].config(text=f"Start {name}")

    def run_ros_stack(self, name):
        config = ROBOTS[name]
        try:
            # 1. Establish and store connection
            conn = Connection(host=config["host"], user=config["user"], connect_kwargs={"password": "ubuntu"})
            self.connections[name] = conn
            
            self.log(name, f">> Connected to {name}.")
            self.log(name, ">> [1/2] Launching Bringup...")

            # Combine commands: Run Bringup in background, sleep 20, then run Nav
            # We use 'bash -c' to ensure the backgrounding and sleep logic works across the SSH channel
            combined_script = (
                f"{BRINGUP_CMD} & "
                "sleep 20 && "
                f"echo '>> [2/2] 20s delay finished. Launching Navigation...' && "
                f"{NAV_CMD}"
            )
            
            # pty=True is essential for ROS 2 streaming output
            conn.run(combined_script, pty=True, out_stream=self.ConsoleWriter(self, name))

        except Exception as e:
            self.log(name, f"!! Connection Lost/Error: {e}")
        finally:
            if name in self.connections: del self.connections[name]
            self.root.after(0, lambda: self.buttons[name].config(text=f"Start {name}"))


    class ConsoleWriter(io.TextIOBase):
        """Helper to pipe Fabric stream directly to Tkinter Text widget"""
        def __init__(self, app, name):
            self.app = app
            self.name = name
        def write(self, s):
            self.app.root.after(0, lambda: self.app.log(self.name, s.strip()))
            return len(s)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotApp(root)
    root.mainloop()
