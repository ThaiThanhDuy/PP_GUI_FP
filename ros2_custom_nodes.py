#!/home/robot/robot_lib/bin/python3
import rclpy
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from queue import Queue
import numpy as np
import yaml
from PIL import Image
import subprocess


import RobotConfig as RobConf
import Utilities as Uti


class MapSubcriber(QThread):
    # progress_status = pyqtSignal(str, str)  # status_type, message
    mapping_status = pyqtSignal(bool)
    def __init__(self, progress_status):
        super().__init__()
        self.progress_status = progress_status
        self.twist_msg = Twist()
        self.is_running = False  # Flag to control thread execution
        self.task_queue = Queue()  # Queue for managing tasks
        self.map_folder = RobConf.MAP_FOLDER
        self.default_map_name = Uti.load_ros_config_sql()['default_map_name'] # load default map name from sql
        self.map_pixmap = None  # Map image in pixmap type, use to display to screen
        self.viewing_map = False  # Status of updating map to GUI
        self.map_width, self.map_height = None, None
        self.node = rclpy.create_node('map_viewer')  # Create ROS 2 node
        self.sub = self.node.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.node.get_logger().info("Started!")

    def run(self, ):
        self.is_running = True  # Set the flag to True to start running
        while self.is_running:
            if not self.task_queue.empty():
                task_function, args, kwargs = self.task_queue.get()  # Get the function and its args
                task_function(*args, **kwargs)  # Execute the task with provided args and kwargs
            self.msleep(200)  # Prevent high CPU usage when idle

    def stop(self):
        self.is_running = False
        self.node.destroy_node()
        self.quit()
        self.wait()

    def map_callback(self, data):
        if self.viewing_map:
            self.update_map(data)

    def update_map(self, map_data):
        if map_data:
            try:
                width = map_data.info.width
                height = map_data.info.height

                # Convert the image to a NumPy array
                data = np.array(map_data.data, dtype=np.int8).reshape((width, height))
                self.map_width, self.map_height = width, height
                rgb_matrix = np.zeros((width, height, 3), dtype=np.uint8)
                rgb_matrix[data == -1] = [147, 174, 174]
                rgb_matrix[data == 0] = [255, 255, 255]
                rgb_matrix[data > 0] = [0, 0, 0]

                #Convert numpy array to QImage
                map_image = QImage(rgb_matrix.data, width, height, 3 * width, QImage.Format_RGB888)
                map_image = map_image.mirrored(True, False)  # Mirror the image horizontally

                # Scale the map image
                # factor_w, factor_h =  660 / width*2, 500 / height*2
                # scaled_image = map_image.scaled(width * factor_w, height * factor_h, Qt.KeepAspectRatio)

                # Convert QImage to QPixmap and attach to self.map_pixmap
                self.map_pixmap = QPixmap.fromImage(map_image)
                self.mapping_status.emit(True)
            except Exception as e:
                self.node.get_logger().warn(str(e))
        else:
            self.node.get_logger().warn("No map_data")

    def load_map(self, map_name=None):
        if map_name == None:
            map_name = self.default_map_name
        try:
            # Loading yaml and image map file
            with open(f"{self.map_folder}/{map_name}.yaml", "r") as file:
                map_metadata = yaml.safe_load(file)
            image = Image.open(f"{self.map_folder}/{map_name}.pgm")

            # Extract thresholds from the YAML metadata
            free_threshold = map_metadata.get("free_thresh", 0.196)
            occupied_threshold = map_metadata.get("occupied_thresh", 0.65)

            # Convert the image to a NumPy array
            pgm_data = np.array(image)
            height, width = pgm_data.shape
            rgb_image = np.stack((pgm_data,) * 3, axis=-1)
            map_image = QImage(rgb_image.data, width, height, 3 * width, QImage.Format_RGB888)

            # Convert QImage to QPixmap and attach to self.map_pixmap
            map_pixmap = QPixmap.fromImage(map_image)
            self.map_pixmap = map_pixmap
            self.progress_status.emit("success", "Loaded map!")
        except Exception as e:
            self.map_pixmap = None
            self.progress_status.emit("error", str(e))
            return

    def _perform_save_map(self, map_name, is_set_to_default):
        self.progress_status.emit("starting", "Saving map")
        if map_name is None:
            map_name = self.default_map_name

        command = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", f"{self.map_folder}/{map_name}"]  # Command to save the map
        try:
            subprocess.run(command, check=True)
            if map_name != None and is_set_to_default:
                self.default_map_name = map_name
                Uti.update_ros_config_sql({'default_map_name': map_name})
            self.progress_status.emit("success", "Map saved successfully.")
        except subprocess.CalledProcessError as e:
            self.progress_status.emit("error", str(e))

    def save_map(self, map_name=None, is_set_to_default=True):
        self.task_queue.put((self._perform_save_map, (map_name, is_set_to_default), {}))  # Add the save_map task to the queue

    def stop(self):
        # Stop the thread by setting the flag to False
        self.is_running = False

        # Wait for the thread to exit gracefully
        self.quit()  # Stops the thread loop
        self.wait()  # Waits for the thread to finish completely

        # Destroy the ROS 2 node and shutdown ROS 2
        if self.node is not None:
            self.node.destroy_node()


class CmdVelPublisher(QThread):
    # progress_status = pyqtSignal(str, str)  # status_type, message
    def __init__(self, progress_status):
        super().__init__()
        self.progress_status = progress_status
        self.is_running = False  # Flag to control thread execution
        self.task_queue = Queue()
        self.twist_msg = Twist()
        self.vx = 0
        self.vw = 0
        self.node = rclpy.create_node('cmd_vel_publisher')  # Create ROS 2 node
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.node.get_logger().info("Started!")

    def run(self, ):
        self.is_running = True  # Set the flag to True to start running
        while self.is_running:
            if not self.task_queue.empty():
                task_function, args, kwargs = self.task_queue.get()  # Get the function and its args
                task_function(*args, **kwargs)  # Execute the task with provided args and kwargs
            self.msleep(100)  # Prevent high CPU usage when idle

    def stop(self):
        self.is_running = False
        self.node.destroy_node()
        self.quit()
        self.wait()

    def _perform_update(self, x, z, mode):
        self.twist_msg.linear.x = float(x)
        self.twist_msg.angular.z = float(z)
        self.twist_msg.linear.y = float(mode)
        try:
            self.pub.publish(self.twist_msg)  # Publish the Twist message
            self.node.get_logger().info(f"Published linear vel: {x}, angular vel: {z}, mode: {mode}")
        except Exception as e:
            self.progress_status.emit('error', str(e))

    def update(self, x, z, mode=1):
        self.task_queue.put((self._perform_update, (x, z, mode,), {}))

    def forward(self, ):
        self.update(self.vx, 0)

    def backward(self, ):
        self.update(-self.vx, 0)

    def left(self, ):
        self.update(0, self.vw)

    def right(self, ):
        self.update(0, -self.vw)

    def stop_movement(self, ):
        self.update(0, 0)

    def move_with_command(self, command, x, z):
        self.vx, self.vw = float(x), float(z)
        print(command)
        if command == "Up":
            self.forward()
        elif command == "Down":
            self.backward()
        elif command == "Left":
            self.left()
        elif command == "Right":
            self.right()
        else:
            self.stop_movement()

    def stop(self):
        # Stop the thread by setting the flag to False
        self.is_running = False

        # Wait for the thread to exit gracefully
        self.quit()  # Stops the thread loop
        self.wait()  # Waits for the thread to finish completely

        # Destroy the ROS 2 node and shutdown ROS 2
        if self.node is not None:
            self.node.destroy_node()


class AmclPoseListener(QThread):
    # progress_status = pyqtSignal(str, str)  # status_type, message
    def __init__(self, progress_status):
        super().__init__()
        self.progress_status = progress_status
        self.is_running = False  # Flag to control thread execution
        self.data_odom = [0.0, 0.0, 0.0, 0.0]
        self.node = rclpy.create_node('amcl_pose_listener')
        # Subscribe to the amcl_pose topic
        self.sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.node.get_logger().info("Started!")

    def run(self, ):
        self.is_running = True  # Set the flag to True to start running

    def pose_callback(self, msg):
        if msg:
            # Extract the position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # Extract the orientation quaternion
            q = msg.pose.pose.orientation

            # siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            # cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            # theta = float(np.atan2(siny_cosp, cosy_cosp))  # Yaw angle in radians
            (roll, pitch, yaw) = euler_from_quaternion((q.x, q.y, q.z, q.w))
            self.data_odom = [x, y, 0.0, yaw]
            # self.node.get_logger().info(f"AMCL Position - x: {x}, y: {y}, theta: {yaw}")
        else:
            self.node.get_logger().warn("No odom data received")

    def stop(self):
        # Stop the thread by setting the flag to False
        self.is_running = False

        # Wait for the thread to exit gracefully
        self.quit()  # Stops the thread loop
        self.wait()  # Waits for the thread to finish completely

        # Destroy the ROS 2 node and shutdown ROS 2
        if self.node is not None:
            self.node.destroy_node()


class GoalPosePublisher(QThread):
    # progress_status = pyqtSignal(str, str)  # status_type, message
    reached_goal = pyqtSignal(bool)
    def __init__(self, progress_status):
        super().__init__()
        self.progress_status = progress_status
        self.is_running = False  # Flag to control thread execution
        self.task_queue = Queue()
        self.node = rclpy.create_node('goal_pose_publisher')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.node.get_logger().info("Started!")
        self.send_goal_result = None
        self.delay_time = Uti.load_ros_config_sql()['delay_time']

    def run(self, ):
        self.is_running = True
        while self.is_running:
            if not self.task_queue.empty():
                task_function, args, kwargs = self.task_queue.get()  # Get the function and its args
                task_function(*args, **kwargs)  # Execute the task with provided args and kwargs
            self.msleep(200)  # Prevent high CPU usage when idle

    def _perform_send_goal(self, x, y, z, theta):
        self.send_goal_result = None
        self.node.get_logger().info(f"Sending goal: x={x}, y={y}, theta={theta}")

        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return
        qx, qy, qz, qw = quaternion_from_euler(0, 0, float(theta))
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
       

        # Send goal
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goal(self, x, y, z, w):
        self.task_queue.put((self._perform_send_goal, (x, y, z, w,), {}))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected.')
            return

        self.node.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        self.send_goal_result = future.result().result
        try:
            if self.send_goal_result:
                self.node.get_logger().info('Goal reached!')
                # self.reached_goal.emit(True)
            else:
                self.node.get_logger().info('Goal was aborted.')
                # self.reached_goal.emit(False)
        except:
            self.send_goal_result = None
            self.node.get_logger().warn('Can get result from future result')

    def _perform_set_goal_from_sql(self, id):
        try:
            x_sql, y_sql, z_sql, w_sql, idout = Uti.nhandulieu_mysql(idban=id)
            self.node.get_logger().info(f'Sending goal from sql by id {id}')
            self._perform_send_goal(x_sql, y_sql, z_sql, w_sql)
        except Exception as e:
            self.progress_status.emit('error', str(e))

    def set_goal_from_sql(self, id):
        self.task_queue.put((self._perform_set_goal_from_sql, (id,), {}))

    def _perform_set_multiple_goals(self, goal_ids):
        for goal_id in goal_ids:
            self._perform_set_goal_from_sql(goal_id)
            while self.send_goal_result is None:
                self.msleep(100)

            if goal_id == 200:
                Uti.RobotSpeakWithPath('hoanthanh.wav')
            else:
                Uti.RobotSpeakWithPath("welcome_voice_temp.mp3")

            self.sleep(self.delay_time)

    def set_multiple_goals(self, goal_ids):
        self.task_queue.put((self._perform_set_multiple_goals, (goal_ids,), {}))

    def stop(self):
        # Stop the thread by setting the flag to False
        self.is_running = False

        # Wait for the thread to exit gracefully
        self.quit()  # Stops the thread loop
        self.wait()  # Waits for the thread to finish completely

        # Destroy the ROS 2 node and shutdown ROS 2
        if self.node is not None:
            self.node.destroy_node()