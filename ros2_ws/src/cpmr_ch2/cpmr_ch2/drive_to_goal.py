import math
from math import atan2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import cv2
import random
from scipy.spatial import KDTree
import time
list12 = []
numberOfNodes = 1000
map = cv2.imread('map.jpg')

purple = (86,70,115)
black = (38,22,27)
darkBlue = (64,38,42)
orange = (68,149,242)
red = (70,78,166)



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


class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self.cylinderList = []

        self._cur_x = 0.0
        self._cur_y= 0.0
        self._goal_x = 24.0
        self._goal_y = 24.0
        self._goal_t = 0.0
        self.max_vel = 0.2
        self.max_gain = 5.0
        self.newGoal= "0.0&0.0"
        self.createMap = "a"

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._goal_x)
        self.declare_parameter('goal_y', value=self._goal_y)
        self.declare_parameter('goal_t', value=self._goal_t)
        self.declare_parameter('max_vel', value=self.max_vel)
        self.declare_parameter('max_gain', value=self.max_gain)
        self.declare_parameter('newGoal', value=self.newGoal)
        self.max_vel = 3.0
        self.max_gain = 5.0

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)
 
    def _listener_callback(self, msg, vel_gain=5.0, max_vel=5.2, max_pos_err=0.05):
       
        vel_gain = self.max_gain
        max_vel = self.max_vel
        pose = msg.pose.pose
        cur_x = pose.position.x
        cur_y = pose.position.y

        self._cur_x = cur_x
        self._cur_y = cur_y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        cur_t = yaw
   

        x_diff = self._goal_x - cur_x
        y_diff = self._goal_y - cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        angleToGoal = atan2(y_diff,x_diff)
        twist = Twist()


        if dist > max_pos_err: 

            vector = [x_diff * vel_gain, y_diff * vel_gain]

            mag = math.sqrt(vector[0]**2 + vector[1]**2 )
            if mag > max_vel:
                vector[0] /= mag
                vector[1] /= mag

                vector[0] *= max_vel
                vector[1] *= max_vel

            x = vector[0]
            y = vector[1]

            twist.linear.x = x * math.cos(cur_t) + y * math.sin(cur_t)
            twist.linear.y = -x * math.sin(cur_t) + y * math.cos(cur_t)
        self._publisher.publish(twist)

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback')
        changedGoal = False
        for param in params:
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_x = param.value
                changedGoal = True
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_y = param.value
                changedGoal = True
            elif param.name == 'goal_t' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_t = param.value
                changedGoal = True
            elif param.name == 'max_vel' and param.type_ == Parameter.Type.DOUBLE:
                self.max_vel = param.value
            elif param.name == 'max_gain' and param.type_ == Parameter.Type.DOUBLE:
                self.max_gain = param.value
            elif param.name == 'newGoal' and param.type_ == Parameter.Type.STRING:
                self.get_logger().warn(f'NEW GOAL IS WORKING S')
                self._goal_x = float(param.value.split("&")[0])
                self._goal_y = float(param.value.split("&")[1])
                changedGoal = True
            
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
           
            if changedGoal:
                self.get_logger().warn(f"Changing goal {self._goal_x} {self._goal_y} {self._goal_t}")
            else:
                self.get_logger().warn(f"Max Velocity Change: {self.max_vel} Max gain changed: {self.max_gain}")
        return SetParametersResult(successful=True)

import os
import cv2
def main(args=None):
   
    rclpy.init(args=args)
    node = MoveToGoal()
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
if __name__ == '__main__':
    main()

