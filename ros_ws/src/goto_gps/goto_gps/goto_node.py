"""
Run:
  ros2 run goto_gps goto_gps --ros-args -p target_lat:=x -p target_lon:=-y
"""

import math

import rclpy
from aqua2_interfaces.msg import AutopilotCommand
from aqua2_interfaces.srv import SetString
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_srvs.srv import SetBool

EARTH_RADIUS_M = 6371000.0


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
    """Constrain angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

#some math I found online
def get_distance_and_bearing(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = phi2 - phi1
    dlam = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    distance = 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))

    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    bearing = math.atan2(y, x)
    return distance, bearing


class GoToGPS(Node):
    def __init__(self):
        super().__init__("goto_gps")

        #parameters
        self.declare_parameter("target_lat", float("nan"))
        self.declare_parameter("target_lon", float("nan"))
        self.declare_parameter("max_speed", 0.1)
        self.declare_parameter("min_speed", 0.05)       # floor so it doesn't stall
        self.declare_parameter("tolerance", 0.5) #how close to a waypoint does the aqua need to get
        self.declare_parameter("slowdown_radius", 10.0) # ramp speed down when this close
        self.declare_parameter("target_depth", 0.0)

        self.target_lat = self.get_parameter("target_lat").value
        self.target_lon = self.get_parameter("target_lon").value
        self.max_speed = self.get_parameter("max_speed").value
        self.min_speed = self.get_parameter("min_speed").value
        self.tolerance = self.get_parameter("tolerance").value
        self.slowdown_radius = self.get_parameter("slowdown_radius").value
        self.target_depth = self.get_parameter("target_depth").value

        self.get_logger().info(f"Target ({self.target_lat}, {self.target_lon}), max_speed {self.max_speed}, arrival_radius {self.tolerance} m")

        # State
        self.lat = None
        self.lon = None
        self.current_yaw = None
        self.timer = None

        self.safe_service = '/a14/dessert/make_safe'
        self.ready_service = '/a14/dessert/make_ready'

        # service clients
        self.config_path = ""
        self.cli_ready = self.create_client(SetString, self.ready_service)
        self.cli_safe = self.create_client(SetBool, self.safe_service)

        # Subscribers
        self.sub_gps = self.create_subscription(NavSatFix, "/a14/gps/fix", self.gps_callback, 1)
        self.sub_imu = self.create_subscription(Imu, "/a14/imu/filtered_data", self.imu_callback, 1)

        # Publishers
        self.pub_autopilot = self.create_publisher(AutopilotCommand, "/a14/autopilot/command", 1)

        self.timer_autopilot = self.create_timer(timer_period_sec=0.1, callback=self.autopilot_callback)


    def wait_for_ready(self):
        """
        Blocks execution until the /make_ready service is available and called.
        """
        self.get_logger().info(f"Waiting for service: {self.ready_service}...")
        while not self.cli_ready.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.get_logger().info(
            f'Service found. Sending "make_ready" request (config: "{self.config_path}")...'
        )

        req = SetString.Request()
        req.value = self.config_path

        future = self.cli_ready.call_async(req)
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
                self.get_logger().error(f"Service {self.safe_service} not found.")


    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.lat = msg.latitude
        self.lon = msg.longitude


    def imu_callback(self, msg: Imu):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_yaw = yaw


    def autopilot_callback(self):
        if self.current_yaw is None:
            return

        if self.lat is None:
            return
        
        distance, bearing = get_distance_and_bearing(self.lat, self.lon, self.target_lat, self.target_lon)

        if distance <= self.tolerance:
            self.publish_command(surge=0.0, yaw_deg=math.degrees(self.current_yaw))
            self.get_logger().info(f"Arrived at ({self.target_lat}, {self.target_lon})")
            return


        desired_yaw = wrap_angle_rad(bearing)

        heading_error = wrap_angle_rad(desired_yaw - self.current_yaw)

        # sort of a capped p controller so we don't drive off too far in the wrong direction
        distance_scale = min(1.0, distance / self.slowdown_radius)
        heading_scale = max(0.0, math.cos(heading_error))
        surge = self.max_speed * distance_scale * heading_scale
        if heading_scale > 0.0:
            surge = max(surge, self.min_speed)

        self.publish_command(surge=surge, yaw_deg=math.degrees(desired_yaw))
        self.get_logger().info(f"dist {distance} m, bearing {math.degrees(bearing)} deg, heading err {math.degrees(heading_error)} deg, surge {surge}")


    def publish_command(self, surge, yaw_deg):
        msg = AutopilotCommand()
        msg.surge = float(surge)
        msg.target_depth = float(self.target_depth)
        msg.target_yaw = float(yaw_deg)
        msg.target_roll = 0.0
        msg.target_pitch = 0.0
        self.pub_autopilot.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    goto_gps = GoToGPS()

    try:
        goto_gps.wait_for_ready()
        rclpy.spin(goto_gps)

    except (KeyboardInterrupt, SystemExit):
        # This catches Ctrl+C OR the SystemExit raised in timer_callback
        goto_gps.wait_for_safe()
    finally:
        if rclpy.ok():
            goto_gps.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()