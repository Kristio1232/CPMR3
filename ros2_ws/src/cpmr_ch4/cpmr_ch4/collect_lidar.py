import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import cv2
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class CollectLidar(Node):
    _WIDTH = 513
    _HEIGHT = 513
    _M_PER_PIXEL = 0.05
    
    
    cur_x = 0
    cur_y = 0
    cur_t = 0

    def __init__(self):
        super().__init__('collect_lidar')
        self.get_logger().info(f'{self.get_name()} created')

        self._map = np.zeros((CollectLidar._HEIGHT, CollectLidar._WIDTH), dtype=np.uint8)
        self.get_logger().info(f"map init {self._map}")

        self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 1)



    def _scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        # self.get_logger().info(f"Ranges ({ranges})\n\n")
        self.get_logger().info(f"lidar ({angle_min},{angle_max},{angle_increment},{len(ranges)})")
        
        for r in enumerate(ranges):
            # self.get_logger().info(f"Range {r}")
            scan_t = self.cur_t + angle_min + r[0] * angle_increment 
            if r[1] < 10:
                self.get_logger().info(f"Range {r}")
                row = int(int(self._HEIGHT/2) - (self.cur_y + r[1] * math.sin(scan_t)) / self._M_PER_PIXEL)
                col = int(int(self._WIDTH/2) + (self.cur_x + r[1] * math.cos(scan_t)) / self._M_PER_PIXEL)
                self.get_logger().info(f"Map {row} {col}")
                if (row >= 0) and (col >= 0) and (row < self._HEIGHT) and (col < self._WIDTH):
                    self._map[row, col] = 200

        cv2.imshow('map',self._map)
        cv2.waitKey(10)

    def _odom_callback(self, msg):
        pose = msg.pose.pose

        self.cur_x = pose.position.x
        self.cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        self.cur_t = yaw
        
        self.get_logger().info(f"at ({self.cur_x},{self.cur_y},{self.cur_t})")

def main(args=None):
    rclpy.init(args=args)
    node = CollectLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
