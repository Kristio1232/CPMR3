from enum import Enum
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

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


class FSM_STATES(Enum):
    AT_START = 'AT STart',
    HEADING_TO_TASK = 'Heading to Task',
    RETURNING_FROM_TASK = 'Returning from Task',
    TASK_DONE = 'Task Done'

class FSM(Node):

    def __init__(self):
        super().__init__('FSM')
        self.get_logger().info(f'{self.get_name()} created')

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        # the blackboard
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._cur_state = FSM_STATES.AT_START
        self._start_time = self.get_clock().now().nanoseconds * 1e-9

    def _drive_to_goal(self, goal_x, goal_y, goal_theta,
                       heading0_tol = 0.1,
                       heading0_v = 0.05,
                       range_tol = 0.1,
                       range_v = 0.1,
                       heading1_tol = 0.1,
                       heading1_v = 0.05):
        """Return True iff we are at the goal, otherwise drive there"""

        self.get_logger().info(f'{self.get_name()} drive to goal')
        twist = Twist()

        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        # turn to the goal
        heading = math.atan2(y_diff, x_diff)
        diff = heading - self._cur_theta
        if diff > math.pi:
            diff = diff - 2 * math.pi
        if diff < -math.pi:
            diff = diff + 2 * math.pi
        if abs(diff) > heading0_tol:
            twist.angular.z = math.copysign(heading0_v, diff)
            self.get_logger().info(f'{self.get_name()} turning towards goal diff {diff} {twist.angular.z}')
            self._publisher.publish(twist)
            return False

        # pointing the right direction, so go there
        if dist > range_tol:
            twist.linear.x = range_v
            self._publisher.publish(twist)
            self.get_logger().info(f'{self.get_name()} driving to goal dist {dist} {twist.linear.x}')
            return False

        # we are there, set the correct angle
        diff = goal_theta - self._cur_theta
        if diff > math.pi:
            diff = diff - 2 * math.pi
        if diff < -math.pi:
            diff = diff + 2 * math.pi
        if abs(diff) > heading1_tol:
            twist.angular.z = math.copysign(heading1_v, diff)
            self.get_logger().info(f'{self.get_name()} turning to goal {goal_theta} {twist.angular.z}')
            self._publisher.publish(twist)
            return False

        self.get_logger().info(f'{self.get_name()} at goal pose')
        self._publisher.publish(twist)
        return True


    def _do_state_at_start(self):
        self.get_logger().info(f'{self.get_name()} in start state')
        now = self.get_clock().now().nanoseconds * 1e-9
        if now > (self._start_time + 2):
            self._cur_state = FSM_STATES.HEADING_TO_TASK

    def _do_state_heading_to_task(self):
        self.get_logger().info(f'{self.get_name()} heading to task {self._cur_x} {self._cur_y} {self._cur_theta}')
        if self._drive_to_goal(2, 1, math.pi/2, heading1_tol = math.pi * 2): # don't care about final orientation
            self._cur_state = FSM_STATES.RETURNING_FROM_TASK

    def _do_state_returning_from_task(self):
        self.get_logger().info(f'{self.get_name()} returning from task ')
        if self._drive_to_goal(0, 0, 0):
            self._cur_state = FSM_STATES.TASK_DONE

    def _do_state_task_done(self):
        self.get_logger().info(f'{self.get_name()} task done')

    def _state_machine(self):
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.HEADING_TO_TASK:
            self._do_state_heading_to_task()
        elif self._cur_state == FSM_STATES.RETURNING_FROM_TASK:
            self._do_state_returning_from_task()
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()
        else:
            self.get_logger().info(f'{self.get_name()} bad state {state_cur_state}')

    def _listener_callback(self, msg):
        pose = msg.pose.pose

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = yaw
        self._state_machine()



def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
