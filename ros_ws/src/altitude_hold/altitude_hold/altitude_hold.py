import math

import rclpy
from aqua2_interfaces.msg import AutopilotCommand
from aqua2_interfaces.srv import SetString
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import SetBool


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw) in radians.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = min(t2, +1.0)
    t2 = max(t2, -1.0)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def wrap_angle_rad(angle):
    """
    Constrain angle to [-pi, pi] range.
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


class AltitudeHold(Node):
    def __init__(self):
        super().__init__("altitude_hold")

        # Set the Aqua's target altitude
        self.declare_parameter("target_altitude", 0.5)
        self.target_altitude = (
            self.get_parameter("target_altitude").get_parameter_value().value
        )
        self.get_logger().info(f"Target altitude set to: {self.target_altitude}")

        # Set the Aqua's maximum speed
        self.declare_parameter("max_speed", 0.1)
        self.max_speed = self.get_parameter("max_speed").get_parameter_value().value
        self.get_logger().info(f"Max speed set to: {self.target_altitude}")

        # Make ready client
        self.config_path = ""
        self.cli_ready = self.create_client(SetString, "/a14/dessert/make_ready")

        # Make safe client
        self.cli_safe = self.create_client(SetBool, "/a14/dessert/make_safe")

        # Subscribe to DVL altitude
        self.current_altitude = None
        self.sub_altitude = self.create_subscription(
            Float32, "/a14/altitude", self.altitude_callback, 1
        )

        # Subscribe to pressure sensor depth
        self.current_depth = None
        self.sub_depth = self.create_subscription(
            Odometry, "/a14/depth", self.depth_callback, 1
        )

        # Subscribe to the IMU heading
        self.current_yaw = None
        self.sub_imu = self.create_subscription(
            Imu, "/a14/imu/filtered_data", self.imu_callback, 1
        )

        # Publish to the Aqua autopilot command
        self.pub_autopilot = self.create_publisher(
            AutopilotCommand, "/a14/autopilot/command", 1
        )

        # Create timer callback for autopilot command
        self.timer_autopilot = self.create_timer(
            timer_period_sec=0.1, callback=self.autopilot_callback
        )

    def wait_for_ready(self):
        """
        Blocks execution until the /make_ready service is available and called.
        """
        self.get_logger().info(f"Waiting for service: {self.service_name}...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.get_logger().info(
            f'Service found. Sending "make_ready" request (config: "{self.config_path}")...'
        )

        req = SetString.Request()
        req.value = self.config_path

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            self.get_logger().info(f'Ready! Response: "{response.msg}"')
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            raise SystemExit

    def wait_for_safe(self):
        self.get_logger().warn("Shutting down... Initiating safety sequence.")

        if self.cli_safe.wait_for_service(timeout_sec=2.0):
            req_safe = SetBool.Request()
            req_safe.data = True
            future_safe = self.cli_safe.call_async(req_safe)

            rclpy.spin_until_future_complete(self, future_safe, timeout_sec=5.0)

            if future_safe.done():
                try:
                    res = future_safe.result()
                    self.get_logger().info(f"Robot Safe: {res.message}")
                except Exception as e:
                    self.get_logger().error(f"Safety call failed: {e}")
        else:
            self.get_logger().error(f"Service {self.safe_service_name} not found.")

    def altitude_callback(self, msg: Float32):
        self.current_altitude = msg.data

    def depth_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        _, _, yaw = euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )
        self.current_yaw = yaw

    def get_target_depth(self):
        if self.current_depth is None:
            return None

        if self.current_altitude is None:
            return None

        surface_to_seafloor = self.current_depth + self.current_altitude
        target_depth = surface_to_seafloor - self.target_altitude

        return target_depth

    def get_target_speed(self):
        return self.max_speed

    def get_target_yaw_deg(self):
        if self.current_yaw is None:
            return None

        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw

        target_yaw_rad = wrap_angle_rad(self.initial_yaw)
        target_yaw_deg = math.degrees(target_yaw_rad)

        return target_yaw_deg

    def autopilot_callback(self):
        target_depth = self.get_target_depth()
        if target_depth is None:
            return

        target_speed = self.get_target_speed()
        if target_speed is None:
            return

        target_yaw_deg = self.get_target_yaw_deg()
        if target_yaw_deg is None:
            return

        # Build the autopilot command
        autopilot_msg = AutopilotCommand()
        autopilot_msg.surge = float(target_speed)
        autopilot_msg.target_depth = float(target_depth)
        autopilot_msg.target_yaw = float(target_yaw_deg)
        autopilot_msg.target_roll = 0.0
        autopilot_msg.target_pitch = 0.0

        self.pub_autopilot.publish(autopilot_msg)


def main(args=None):
    rclpy.init(args=args)

    altitude_hold = AltitudeHold()

    try:
        altitude_hold.wait_for_ready()
        altitude_hold.start_test()
        rclpy.spin(altitude_hold)

    except (KeyboardInterrupt, SystemExit):
        # This catches Ctrl+C OR the SystemExit raised in timer_callback
        altitude_hold.wait_for_safe()
    finally:
        if rclpy.ok():
            altitude_hold.destroy_node()
            rclpy.shutdown()

    altitude_hold.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
