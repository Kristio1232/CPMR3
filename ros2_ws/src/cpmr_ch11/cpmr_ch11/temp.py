class FSM(Node):

    def __init__(self):
        super().__init__('FSM')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('chair_name', "chair_0")
        chair_name = self.get_parameter('chair_name').get_parameter_value().string_value

        self.create_subscription(Odometry, f"/{chair_name}/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, f"/{chair_name}/cmd_vel", 1)
        self.create_service(SetBool, f"/{chair_name}/startup", self._startup_callback)
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_id = 0

        # Blackboard variables
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._cur_state = FSM_STATES.AT_START
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._points = [[10, 0], [10, 10], [15, 10], [15, 0]]
        self._point = 0
        self._run = False

    def _drive_to_goal(self, goal_x, goal_y,
                       heading0_tol=0.1,    # Reduced heading tolerance for precision
                       range_tol=0.15):
        """Return True iff we are at the goal, otherwise drive there"""

        twist = Twist()
        
        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        
        if dist > range_tol:
            self.get_logger().info(f'{self.get_name()} driving to goal with goal distance {dist}')
            # Turn towards the goal
            heading = math.atan2(y_diff, x_diff)
            diff = FSM._short_angle(heading - self._cur_theta)

            if abs(diff) > heading0_tol:
                twist.angular.z = FSM._compute_speed(diff, 0.7, 0.05, 0.7) # Increased angular speed gain
                self.get_logger().info(f'{self.get_name()} turning towards goal heading {heading} current {self._cur_theta} diff {diff} {twist.angular.z}')
                self._publisher.publish(twist)
                return False

            twist.linear.x = FSM._compute_speed(dist, 0.7, 0.1, 0.2) # Increased max speed and decreased min speed
            self._publisher.publish(twist)
            self.get_logger().info(f'{self.get_name()} a distance {dist} from target velocity {twist.linear.x}')
            return False

        # Stop if at goal
        twist.linear.x = twist.angular.z = 0.0 
        self.get_logger().info(f'at goal pose')
        self._publisher.publish(twist)
        return True

    # Other methods remain unchanged...