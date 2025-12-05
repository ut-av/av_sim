#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import json
import struct
import time
import threading
import math
import base64
import cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import LaserScan, Imu, Image, Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool
from amrl_msgs.msg import AckermannCurvatureDriveMsg
from ut_automata.msg import VescStateStamped, CarStatusMsg
import tf2_ros

class SimulatorBridge(Node):
    def __init__(self):
        super().__init__('simulator_bridge')

        # Parameters
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 9999)
        self.declare_parameter('retry_interval', 2.0)
        
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.retry_interval = self.get_parameter('retry_interval').value

        # Joystick Parameters
        self.declare_parameter('joystick_mode', 'both')
        self.declare_parameter('joystick_turbo_speed', 5.0)
        self.declare_parameter('joystick_normal_speed', 2.0)
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('steering_curve_xm', 0.5)
        self.declare_parameter('steering_curve_ym', 0.5)

        self.joystick_mode = self.get_parameter('joystick_mode').value
        self.joystick_turbo_speed = self.get_parameter('joystick_turbo_speed').value
        self.joystick_normal_speed = self.get_parameter('joystick_normal_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_curve_xm = self.get_parameter('steering_curve_xm').value
        self.steering_curve_ym = self.get_parameter('steering_curve_ym').value

        # Load config from Lua files
        self.load_config()

        # State for joystick
        self.drive_mode = "stopped" # stopped, joystick, autonomous
        self.last_joystick_time = 0.0

        # TCP Connection
        self.sock = None
        self.connected = False
        self.buffer = ""
        self.last_connection_attempt = 0.0

        # ROS Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.image_pub = self.create_publisher(Image, '/camera_0/image_raw', 10)
        self.vesc_state_pub = self.create_publisher(VescStateStamped, '/sensors/core', 10)
        self.car_status_pub = self.create_publisher(CarStatusMsg, '/car_status', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ROS Subscribers
        self.create_subscription(AckermannCurvatureDriveMsg, '/ackermann_curvature_drive', self.control_callback, 10)
        self.create_subscription(Joy, '/joystick', self.joystick_callback, 10)

        # Helpers
        self.cv_bridge = CvBridge()

        # Connect to simulator
        self.connect_to_simulator()

        # Start receive loop
        self.timer = self.create_timer(0.01, self.receive_data)

    def load_config(self):
        """Load configuration from Lua files."""
        import os
        
        config_files = [
            '/home/ntsoi/roboracer_ws/src/ut_automata/config/vesc.lua',
            '/home/ntsoi/roboracer_ws/src/ut_automata/config/car.lua'
        ]
        
        config = {}
        
        for file_path in config_files:
            if not os.path.exists(file_path):
                self.get_logger().info(f"Config file not found (skipping): {file_path}")
                continue
                
            self.get_logger().info(f"Loading config from: {file_path}")
            try:
                with open(file_path, 'r') as f:
                    for line in f:
                        # Strip comments
                        line = line.split('--')[0].strip()
                        if not line:
                            continue
                        
                        # Parse key = value;
                        if '=' in line and ';' in line:
                            parts = line.split('=')
                            key = parts[0].strip()
                            value_str = parts[1].split(';')[0].strip()
                            
                            # Try to parse value
                            try:
                                value = float(value_str)
                            except ValueError:
                                # Handle strings or booleans if needed
                                if value_str.lower() == 'true':
                                    value = True
                                elif value_str.lower() == 'false':
                                    value = False
                                else:
                                    value = value_str.strip('"')
                            
                            config[key] = value
            except Exception as e:
                self.get_logger().error(f"Error reading config file {file_path}: {e}")

        # Apply config to parameters
        if 'joystick_turbo_speed' in config:
            self.joystick_turbo_speed = float(config['joystick_turbo_speed'])
            self.get_logger().info(f"Set joystick_turbo_speed to {self.joystick_turbo_speed}")
            
        if 'joystick_normal_speed' in config:
            self.joystick_normal_speed = float(config['joystick_normal_speed'])
            self.get_logger().info(f"Set joystick_normal_speed to {self.joystick_normal_speed}")
            
        if 'max_steering_angle' in config:
            self.max_steering_angle = float(config['max_steering_angle'])
            self.get_logger().info(f"Set max_steering_angle to {self.max_steering_angle}")
            
        if 'steering_curve_xm' in config:
            self.steering_curve_xm = float(config['steering_curve_xm'])
            self.get_logger().info(f"Set steering_curve_xm to {self.steering_curve_xm}")
            
        if 'steering_curve_ym' in config:
            self.steering_curve_ym = float(config['steering_curve_ym'])
            self.get_logger().info(f"Set steering_curve_ym to {self.steering_curve_ym}")

    def connect_to_simulator(self):
        """Attempt to connect to the simulator. Non-blocking - returns True if successful."""
        try:
            # Close existing socket if present
            if self.sock is not None:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None
            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)  # 2 second timeout for connection attempt
            self.sock.connect((self.host, self.port))
            self.connected = True
            self.get_logger().info(f"Connected to simulator at {self.host}:{self.port}")
            
            # Reset state on successful connection
            self.reset_state()
            
            return True
            
        except Exception as e:
            self.get_logger().debug(f"Failed to connect to simulator: {e}")
            self.connected = False
            if self.sock is not None:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None
            return False

    def reset_state(self):
        """Reset internal state and send handshake to simulator."""
        # Clear receive buffer
        self.buffer = ""
        
        # Send initial handshake to simulator
        self.send_json({"msg_type": "car_loaded"})
        self.get_logger().info("State reset and handshake sent to simulator")
        
        # Send camera config
        self.send_camera_config()

    def send_camera_config(self):
        """Send camera configuration to simulator."""
        msg = {
            "msg_type": "cam_config",
            "fov": "90",
            "fish_eye_x": "0.0",
            "fish_eye_y": "0.0",
            "img_w": "320",
            "img_h": "240",
            "img_d": "3",
            "img_enc": "JPG",
            "img_quality": "100",
            "offset_x": "0.0",
            "offset_y": "0.0",
            "offset_z": "0.0",
            "rot_x": "0.0",
            "rot_y": "0.0",
            "rot_z": "0.0",
            "super_sampling": "4",
            "anti_aliasing": "4",
        }
        self.send_json(msg)
        self.get_logger().info("Sent camera configuration")

    def send_json(self, data):
        if not self.connected:
            return
        try:
            msg = json.dumps(data)
            self.sock.sendall(msg.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")
            self.connected = False

    def receive_data(self):
        if not self.connected:
            # Try to reconnect periodically
            current_time = time.time()
            if current_time - self.last_connection_attempt >= self.retry_interval:
                self.last_connection_attempt = current_time
                self.get_logger().info(f"Attempting to reconnect to simulator at {self.host}:{self.port}...")
                self.connect_to_simulator()
            return

        try:
            # Non-blocking receive - read ALL available data
            self.sock.setblocking(0)
            while True:
                try:
                    data = self.sock.recv(4096).decode('utf-8')
                    if not data:
                        self.connected = False
                        self.get_logger().warn("Disconnected from simulator")
                        # Clean up socket to prepare for reconnection
                        if self.sock is not None:
                            try:
                                self.sock.close()
                            except:
                                pass
                            self.sock = None
                        return
                    self.buffer += data
                except BlockingIOError:
                    break # No more data available right now

            # Process buffer
            messages = []
            while '}' in self.buffer:
                start = self.buffer.find('{')
                if start == -1:
                    self.buffer = ""
                    break
                
                brace_count = 0
                end = -1
                for i, char in enumerate(self.buffer[start:], start):
                    if char == '{':
                        brace_count += 1
                    elif char == '}':
                        brace_count -= 1
                        if brace_count == 0:
                            end = i + 1
                            break
                
                if end != -1:
                    json_str = self.buffer[start:end]
                    self.buffer = self.buffer[end:]
                    try:
                        msg = json.loads(json_str)
                        messages.append(msg)
                    except json.JSONDecodeError:
                        self.get_logger().warn(f"Failed to decode JSON: {json_str}")
                else:
                    break # Incomplete message

            # Process messages, keeping only the latest telemetry
            latest_telemetry = None
            for msg in messages:
                msg_type = msg.get("msg_type")
                if msg_type == "telemetry":
                    latest_telemetry = msg
                else:
                    self.process_message(msg)
            
            # Process the latest telemetry if we have one
            if latest_telemetry:
                self.process_message(latest_telemetry)

        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")
            self.connected = False
            # Clean up socket to prepare for reconnection
            if self.sock is not None:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None

    def process_message(self, msg):
        msg_type = msg.get("msg_type")
        if msg_type == "telemetry":
            self.handle_telemetry(msg)
        elif msg_type == "car_loaded":
            self.get_logger().info("Simulator confirmed car loaded")

    def handle_telemetry(self, msg):
        now = self.get_clock().now().to_msg()

        # debug print all message types
        #self.get_logger().info(f"Received telemetry: {msg.keys()}")
        
        # 1. Odometry
        if "pos_x" in msg:
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            
            # Position (Unity is y-up, ROS is z-up. Usually x=z, y=-x, z=y or similar.
            # But let's check standard Unity->ROS conversion.
            # Usually: ROS x = Unity z, ROS y = Unity -x, ROS z = Unity y
            # But let's look at the data. 
            # For now assuming direct mapping or simple swap.
            # Let's trust the simulator sends standard coordinates if possible, 
            # or we might need to adjust.
            # Based on TcpCarHandler.cs:
            # json.AddField("pos_x", pos.x);
            # json.AddField("pos_y", pos.y);
            # json.AddField("pos_z", pos.z);
            # Unity: x right, y up, z forward.
            # ROS: x forward, y left, z up.
            # So: ROS x = Unity z, ROS y = -Unity x, ROS z = Unity y.
            
            odom.pose.pose.position.x = float(msg["pos_z"])
            odom.pose.pose.position.y = -float(msg["pos_x"])
            odom.pose.pose.position.z = float(msg["pos_y"])
            
            # Orientation (Quaternion)
            # Unity: x, y, z, w
            # We need to rotate the quaternion frame.
            # For now, let's just pass it through and see if we need to fix it.
            # Actually, let's try to be correct.
            # Q_ros = [-Q_unity.x, Q_unity.z, -Q_unity.y, Q_unity.w] ?
            # Let's stick to simple mapping for now and refine if needed.
            # Actually, let's look at `TcpCarHandler.cs` again.
            # It sends Euler angles too: pitch, yaw, roll.
            # json.AddField("pitch", eulerAngles.x);
            # json.AddField("yaw", eulerAngles.y);
            # json.AddField("roll", eulerAngles.z);
            
            # Let's use the velocity for Twist
            odom.twist.twist.linear.x = float(msg["speed"]) # This is magnitude
            # We also have vel_x, vel_y, vel_z in extended telemetry
            if "vel_x" in msg:
                odom.twist.twist.linear.x = float(msg["vel_z"])
                odom.twist.twist.linear.y = -float(msg["vel_x"])
                odom.twist.twist.linear.z = float(msg["vel_y"])

            self.odom_pub.publish(odom)

            # Publish TF
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            # TODO: Fix rotation
            t.transform.rotation.w = 1.0 
            self.tf_broadcaster.sendTransform(t)

        # UnitySensors style Lidar
        if "lidar_scan" in msg:
            lidar_data = msg["lidar_scan"]
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = "laser"
            
            scan.angle_min = float(lidar_data["min_angle"])
            scan.angle_max = float(lidar_data["max_angle"])
            scan.angle_increment = float(lidar_data["angle_increment"])
            scan.range_min = float(lidar_data["range_min"])
            scan.range_max = float(lidar_data["range_max"])
            
            # Decode ranges
            ranges_b64 = lidar_data["ranges"]
            ranges_bytes = base64.b64decode(ranges_b64)
            # Convert bytes to floats
            # Assuming little-endian (Unity C# uses system endianness, usually little on x86/ARM)
            # struct.unpack expects bytes.
            count = len(ranges_bytes) // 4
            scan.ranges = list(struct.unpack(f'<{count}f', ranges_bytes))
            
            self.scan_pub.publish(scan)

        # old donkey-sim based lidar
        elif "lidar" in msg:
            lidar_data = msg["lidar"]
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = "laser"
            
            # We need to construct a LaserScan from the point cloud data
            # The simulator sends a list of points {d, rx, ry}
            # rx is horizontal angle, ry is vertical.
            # We want to flatten this to 2D scan.
            # We can filter for ry close to 0.
            
            # Assuming standard Hokuyo:
            # angle_min: -2.35619
            # angle_max: 2.35619
            # angle_increment: ?
            
            # Let's infer from data or hardcode standard values
            scan.angle_min = -2.35619
            scan.angle_max = 2.35619
            scan.range_min = 0.1
            scan.range_max = 30.0
            
            # We need to bin the points into the scan array
            # Let's assume 0.25 degree increment (approx 1080 points for 270 deg)
            angle_inc = math.radians(0.25)
            scan.angle_increment = angle_inc
            num_readings = int((scan.angle_max - scan.angle_min) / angle_inc)
            scan.ranges = [float('inf')] * num_readings
            
            for point in lidar_data:
                d = float(point["d"])
                rx = float(point["rx"]) # degrees
                ry = float(point["ry"]) # degrees
                
                # Filter for horizontal plane (approx)
                if abs(ry) < 2.0:
                    # Convert rx to radians and map to index
                    # rx is 0 forward, positive right?
                    # ROS: 0 forward, positive left.
                    angle = math.radians(-rx)
                    
                    if scan.angle_min <= angle <= scan.angle_max:
                        idx = int((angle - scan.angle_min) / angle_inc)
                        if 0 <= idx < num_readings:
                            if d < scan.ranges[idx]:
                                scan.ranges[idx] = d
            
            self.scan_pub.publish(scan)

        # 3. Camera
        if "image" in msg:
            try:
                img_data = base64.b64decode(msg["image"])
                np_arr = np.frombuffer(img_data, np.uint8)
                image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                ros_image = self.cv_bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
                ros_image.header.stamp = now
                ros_image.header.frame_id = "camera"
                self.image_pub.publish(ros_image)
            except Exception as e:
                self.get_logger().warn(f"Failed to process image: {e}")

        # 4. VESC State
        vesc_state = VescStateStamped()
        vesc_state.header.stamp = now
        vesc_state.header.frame_id = "base_link"
        vesc_state.state.speed = float(msg.get("speed", 0.0))
        # vesc_state.state.servo = float(msg.get("steering_angle", 0.0)) # Field does not exist in VescState
        # Actually simulator sends normalized.
        # We might need to map it back if VESC expects something else.
        # But VescState is usually for feedback.
        self.vesc_state_pub.publish(vesc_state)
        
        # 5. Car Status
        car_status = CarStatusMsg()
        car_status.header = vesc_state.header
        # Fill in what we can
        self.car_status_pub.publish(car_status)

        # 6. IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu"
        imu.linear_acceleration.x = float(msg.get("accel_x", 0.0))
        imu.linear_acceleration.y = float(msg.get("accel_y", 0.0))
        imu.linear_acceleration.z = float(msg.get("accel_z", 0.0))
        imu.angular_velocity.x = float(msg.get("gyro_x", 0.0))
        imu.angular_velocity.y = float(msg.get("gyro_y", 0.0))
        imu.angular_velocity.z = float(msg.get("gyro_z", 0.0))
        self.imu_pub.publish(imu)

    def bezier4(self, t, p0, p1, p2, p3, p4):
        one_minus_t = 1.0 - t
        one_minus_t2 = one_minus_t * one_minus_t
        one_minus_t3 = one_minus_t2 * one_minus_t
        one_minus_t4 = one_minus_t3 * one_minus_t
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        
        return (one_minus_t4 * p0 +
                4.0 * one_minus_t3 * t * p1 +
                6.0 * one_minus_t2 * t2 * p2 +
                4.0 * one_minus_t * t3 * p3 +
                t4 * p4)

    def bezier4_prime(self, t, p0, p1, p2, p3, p4):
        one_minus_t = 1.0 - t
        one_minus_t2 = one_minus_t * one_minus_t
        one_minus_t3 = one_minus_t2 * one_minus_t
        t2 = t * t
        t3 = t2 * t
        
        return (4.0 * one_minus_t3 * (p1 - p0) +
                12.0 * one_minus_t2 * t * (p2 - p1) +
                12.0 * one_minus_t * t2 * (p3 - p2) +
                4.0 * t3 * (p4 - p3))

    def joystick_callback(self, msg):
        # Constants
        kTriangleButton = 1
        kL1Button = 4
        kL2Button = 6
        
        if len(msg.buttons) < 6:
            return

        # 1. Reset Logic (Triangle Button)
        if msg.buttons[kTriangleButton] == 1:
            self.reset_state()
            # Debounce or just return to avoid driving while resetting
            return

        # 2. Deadman Switch Logic (L1 Button)
        # If L1 is NOT held, stop the car
        if msg.buttons[kL1Button] == 0:
            # the physics of the car should make it come to a stop automatically
            # self.drive_mode = "stopped"
            # # Send stop command
            # stop_msg = {
            #     "msg_type": "control",
            #     "throttle": "0.0",
            #     "steering": "0.0",
            #     "brake": "1.0"
            # }
            # self.send_json(stop_msg)
            return
        
        # If we are here, L1 is held, so we are driving
        self.drive_mode = "joystick"

        # Check axes
        min_axes = 5
        if self.joystick_mode == "left":
            min_axes = 2
        elif self.joystick_mode == "right":
            min_axes = 5
            
        if len(msg.axes) < min_axes:
            return

        steer_joystick = 0.0
        drive_joystick = 0.0

        if self.joystick_mode == "both":
            steer_joystick = msg.axes[0]
            drive_joystick = -msg.axes[5]
        elif self.joystick_mode == "left":
            steer_joystick = msg.axes[0]
            drive_joystick = -msg.axes[1]
        elif self.joystick_mode == "right":
            steer_joystick = msg.axes[3]
            drive_joystick = -msg.axes[5]
        else:
            # Default to both
            steer_joystick = msg.axes[0]
            drive_joystick = -msg.axes[5]

        # 3. Turbo Logic (L2 Axis)
        # L2 is usually -1.0 (released) to 1.0 (pressed) or 0 to 1 depending on driver?
        # Standard ROS Joy for triggers: 1.0 (released) to -1.0 (pressed) OR -1.0 (released) to 1.0 (pressed).
        # User said "if L2 is also being held".
        # Let's assume standard behavior where pressed is typically positive or negative extreme.
        # Code before used: `turbo_mode = (msg.axes[2] >= 0.9)` which implies pressed is positive.
        # Let's stick to that but also check if axis exists.
        
        turbo_mode = msg.buttons[kL2Button]
        max_speed = self.joystick_turbo_speed if turbo_mode else self.joystick_normal_speed

        speed = drive_joystick * max_speed
        self.get_logger().debug("turbo_mode: %d, max_speed: %f, joystick: %f, speed: %f" % (turbo_mode, max_speed, drive_joystick, speed))

        # Bezier Curve for Steering
        x_target = steer_joystick
        t = (x_target + 1.0) * 0.5
        
        # Newton's method
        for _ in range(10):
            x_val = self.bezier4(t, -1.0, -self.steering_curve_xm, 0.0, self.steering_curve_xm, 1.0)
            error = x_val - x_target
            if abs(error) < 1e-4:
                break
            dx_dt = self.bezier4_prime(t, -1.0, -self.steering_curve_xm, 0.0, self.steering_curve_xm, 1.0)
            if abs(dx_dt) < 1e-6:
                break
            t -= error / dx_dt
            t = max(0.0, min(1.0, t))
            
        steer_curved = self.bezier4(t, -1.0, -self.steering_curve_ym, 0.0, self.steering_curve_ym, 1.0)
        steering_angle = steer_curved * self.max_steering_angle

        # Send to simulator
        self.send_control_command(speed, steering_angle)

    def control_callback(self, msg):
        if self.drive_mode == "joystick":
            return # Ignore autonomous commands in joystick mode

        # Convert AckermannCurvatureDriveMsg to speed/angle
        speed = msg.velocity
        
        # Curvature = 1/R = tan(delta) / L
        # delta = atan(curvature * L)
        wheelbase = 0.33 # m (standard 1/10 scale)
        steering_angle = math.atan(msg.curvature * wheelbase)
        
        self.send_control_command(speed, steering_angle)

    def send_control_command(self, speed, steering_angle):
        # Map speed (m/s) to throttle (-1 to 1)
        # Assuming max speed of simulator car is around 5-10 m/s?
        # Let's use a configurable max speed or just a reasonable constant.
        # The C++ code uses VESC RPM control. Here we send throttle.
        # Let's assume 5 m/s is full throttle for now.
        max_sim_speed = 5.0 
        throttle = speed / max_sim_speed
        throttle = max(min(throttle, 1.0), -1.0)
        
        # Map steering angle (rad) to steering (-1 to 1)
        # Simulator expects -1 (left) to 1 (right) ? Or 0-1?
        # Usually -1 to 1.
        # We need to know the max steering angle of the simulated car.
        # Let's use the parameter we have.
        steering = steering_angle / self.max_steering_angle
        steering = max(min(steering, 1.0), -1.0)
        
        control_msg = {
            "msg_type": "control",
            "throttle": str(throttle),
            "steering": str(steering),
            "brake": "0.0"
        }
        
        self.send_json(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
