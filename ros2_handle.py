#!/home/robot/robot_lib/bin/python3
import sys
import os
import signal
import psutil
import rclpy
from PyQt5.QtCore import QThread, pyqtSignal
from queue import Queue
import yaml
import subprocess

from rclpy.executors import MultiThreadedExecutor

from ros2_custom_nodes import CmdVelPublisher, MapSubcriber, GoalPosePublisher, AmclPoseListener
import RobotConfig as RobConf

class LaunchFileManager(QThread):
    def __init__(self, launch_file_name, progress_status, is_popup_status=True, map_name=None):
        super().__init__()
        self.progress_status = progress_status
        self.is_running = False
        self.package_name = "library_robot2"
        self.launch_file_name = launch_file_name
        self.process = None
        self.task_queue = Queue()
        self.map_name = map_name
        self.is_popup_status = is_popup_status

    def run(self):
        self.is_running = True
        if self.is_popup_status:
            self.progress_status.emit("loading", f"Running {self.launch_file_name}")
        self.process = self.run_launch_file(self.launch_file_name)
        if self.is_popup_status:
            self.progress_status.emit("success", f"Loaded {self.launch_file_name} successfully")

    def run_launch_file(self, launch_file_name):
        is_build = ""
        map_arg = ""
        if launch_file_name == "navigation":
            is_build = f"&& cd ~/ros2_ws/src && colcon build "
            map_arg = f"map_name:={self.map_name}.yaml"
        # Prepare environment variables
        env = os.environ.copy()
        env["ROS_DISTRO"] = "humble"
        env["ROS_WORKSPACE"] = "~/ros2_ws/src/install"  # Adjust to your workspace path
        env["PYTHONUNBUFFERED"] = "1"

        execute_command = f"source /opt/ros/humble/setup.bash && source ~/ros2_ws/src/install/setup.bash {is_build}&& ros2 launch {self.package_name} {launch_file_name}.launch.py {map_arg}"
        command = f"gnome-terminal --geometry=80x24+1000+1000 --title={launch_file_name} -- bash -c '{execute_command}'"
        # Use subprocess to run the command
        process = subprocess.Popen(command, 
                                    shell=True
                                )
        process.wait()
        return process

    def stop_process(self, is_status):
        """
        Stops the terminal running the ROS 2 process safely.
        """
        if self.process and self.process.poll() is None:  # Check if the process is still running
            try:
                # Get the process object using psutil
                ros2_process = psutil.Process(self.process.pid)
                
                # Get the parent process (likely the terminal)
                parent_process = ros2_process.parent()
                
                if parent_process and "gnome-terminal" in parent_process.name():
                    print(f"Killing terminal: {parent_process.name()} (PID: {parent_process.pid})")
                    parent_process.terminate()  # Kill the terminal
                    parent_process.wait()  # Ensure it is terminated
                else:
                    print("Parent process is not a gnome-terminal.")
                
                # Also ensure the ROS 2 process is terminated
                ros2_process.terminate()
                ros2_process.wait()  # Clean up the ROS 2 process
                
                print("ROS 2 terminal and process terminated successfully.")
            except psutil.NoSuchProcess as e:
                print(f"Process not found: {e}")
        else:
            print("ROS 2 process is not running or already terminated.")

    def kill_terminal(self, ):
        try:
            # Use pkill to terminate the terminal by title
            subprocess.run(["pkill", "-f", f"{self.launch_file_name}"], check=True)
            print(f"Terminal named '{self.launch_file_name}' has been killed.")
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")

    def stop(self, is_status=True):
        self.kill_terminal()
        self.quit()
        self.wait()

class ROS2Handle(QThread):
    progress_status = pyqtSignal(str, str)  # status_type, message
    def __init__(self):
        super().__init__()
        self.is_running = False
        self.status = "navigation"
        self.package_name = "library_robot2"
        self.live_robot_launch_file = "live_robot"
        self.navigation_launch_file = "navigation"
        self.mapping_launch_file = "mapping"
        self.live_robot_manager = None
        self.navigation_manager = None
        self.mapping_manager = None

        # Start nodes
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.cmd_vel_publisher = CmdVelPublisher(self.progress_status)
        self.map_viewer_subcriber = MapSubcriber(self.progress_status)
        self.odom_listener = AmclPoseListener(self.progress_status)
        self.goal_publisher = GoalPosePublisher(self.progress_status)
        self.workers = [self.cmd_vel_publisher, self.map_viewer_subcriber, self.odom_listener, self.goal_publisher]
        for worker in self.workers:
            worker.start()

    def run(self):
        self.is_running = True  # Set the flag to True to start running

        # Run live_robot launch file
        self.live_robot_manager = LaunchFileManager(self.live_robot_launch_file, self.progress_status, is_popup_status=False)
        self.live_robot_manager.start()

        # Run navigation launch file
        self.navigation_manager = LaunchFileManager(self.navigation_launch_file, self.progress_status, is_popup_status=False, map_name=self.map_viewer_subcriber.default_map_name)
        self.navigation_manager.start()
        for worker in self.workers:
            self.executor.add_node(worker.node)
        self.executor.spin()

    def start_mapping(self, ):
        self.progress_status.emit("starting", "Start mapping")
        self.map_viewer_subcriber.viewing_map = True
        self.mapping_manager = LaunchFileManager(self.mapping_launch_file, self.progress_status)
        self.mapping_manager.start()
        self.status = "mapping"

    def stop_mapping(self, is_status=True):
        self.map_viewer_subcriber.viewing_map = False
        if self.mapping_manager:
            self.mapping_manager.stop(is_status)
            self.mapping_manager = None

    def start_navigation(self, ):
        self.progress_status.emit("starting", "Start navigation")
        self.navigation_manager = LaunchFileManager(self.navigation_launch_file, self.progress_status, True, self.map_viewer_subcriber.default_map_name)
        self.navigation_manager.start()
        self.status = "navigation"

    def stop_navigation(self, is_status=True):
        if self.navigation_manager:
            self.navigation_manager.stop(is_status)
            self.navigation_manager = None

    def stop(self, is_status=True):
        self.stop_mapping(is_status)
        self.stop_navigation(is_status)
        if self.live_robot_manager:
            self.live_robot_manager.stop(is_status)
        for worker in self.workers:
            worker.stop()
        self.executor.shutdown()  # Cleanly shuts down executor
        try:
            rclpy.shutdown()
        except Exception as e:
            print(e)
        self.quit()
        self.wait()