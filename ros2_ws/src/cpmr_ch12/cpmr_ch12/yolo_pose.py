import os
import sys
import rclpy
import cv2
import datetime
import numpy as np
import pandas as pd
import math
from .arm_helper import robot_arm

from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from ultralytics.engine.results import Results, Keypoints
from ament_index_python.packages import get_package_share_directory

class YOLO_Pose(Node):
    _BODY_PARTS = ["NOSE", "LEFT_EYE", "RIGHT_EYE", "LEFT_EAR", "RIGHT_EAR", "LEFT_SHOULDER", "RIGHT_SHOULDER",
                   "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE",
                   "RIGHT_KNEE", "LEFT_ANKLE", "RIGHT_ANKLE"]
    def __init__(self):
        super().__init__('pose_node')

        # params
        self._model_file = os.path.join(get_package_share_directory('cpmr_ch12'), 'yolov8n-pose.pt') 
        self.declare_parameter("model", self._model_file) 
        model = self.get_parameter("model").get_parameter_value().string_value

        self.declare_parameter("device", "cpu")
        self._device = self.get_parameter("device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self._threshold = self.get_parameter("threshold").get_parameter_value().double_value


        self.declare_parameter("camera_topic", "/mycamera/image_raw")
        self._camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value

        
        self._move_flag = False
        self._bridge = CvBridge()
        self._model = YOLO(model)
        self._model.fuse()

        # subs
        self._sub = self.create_subscription(Image, self._camera_topic, self._camera_callback, 1)
        self.robot_arm = robot_arm()
        
    def parse_keypoints(self, results: Results):

        keypoints_list = []

        for points in results.keypoints:        
            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):
                if conf >= self._threshold:
                    keypoints_list.append([kp_id, p[0], p[1], conf])

        return keypoints_list
    
    def _camera_callback(self, data):
        self.get_logger().info(f'{self.get_name()} camera callback')
        img = self._bridge.imgmsg_to_cv2(data)
        results = self._model.predict(
                source = img,
                verbose = False,
                stream = False,
                conf = self._threshold,
                device = self._device
        )

        if len(results) != 1:
            self.get_logger().info(f'{self.get_name()}  Nothing to see here or too much {len(results)}')
            return
            
        results = results[0].cpu()
        if len(results.boxes.data) == 0:
            self.get_logger().info(f'{self.get_name()}  boxes are too small')
            return

        if results.keypoints:
            keypoints = self.parse_keypoints(results)
            left_shoulder = None
            right_shoulder = None
            if len(keypoints) > 0:
                for i in range(len(keypoints)):
                    self.get_logger().info(f'{self.get_name()}  {YOLO_Pose._BODY_PARTS[keypoints[i][0]]} {keypoints[i]}')
                    if part == "LEFT_SHOULDER":
                        left_shoulder = keypoints[i][1:]
                    elif part == "RIGHT_SHOULDER":
                        right_shoulder = keypoints[i][1:]
                    elif part == "LEFT_WRIST":
                        left_wrist = keypoints[i][1:]
                    elif part == "RIGHT_WRIST":
                        right_wrist = keypoints[i][1:]
                    elif part == "NOSE":
                        nose = keypoints[i][1:]
                    elif part == "LEFT_EYE":
                        left_eye = keypoints[i][1:]
                    elif part == "RIGHT_EYE":
                        right_eye = keypoints[i][1:]

                
                # Visualize results on frame
                # if left_eye and right_eye and (left_shoulder or right_shoulder):
                #     eye_y = (left_eye[1] + right_eye[1]) / 2
                #     if left_shoulder and right_shoulder:
                #         shoulder_y = (left_shoulder[1] + right_shoulder[1]) / 2
                #     elif left_shoulder:
                #         shoulder_y = left_shoulder[1]
                #     else:
                #         shoulder_y = right_shoulder[1]
                #     eye_to_shoulder_distance = abs(eye_y - shoulder_y)

                #     # Check conditions and set goal_x and goal_y
                #     left_arm_above = False
                #     left_arm_below = False
                #     right_arm_above = False
                #     right_arm_below = False

                # if left_wrist and left_shoulder:
                #     if left_wrist[1] < left_shoulder[1] - eye_to_shoulder_distance:
                #         left_arm_above = True
                #     elif left_wrist[1] > left_shoulder[1] + eye_to_shoulder_distance:
                #         left_arm_below = True
                
                # if right_wrist and right_shoulder:
                #     if right_wrist[1] < right_shoulder[1] - eye_to_shoulder_distance:
                #         right_arm_above = True
                #     elif right_wrist[1] > right_shoulder[1] + eye_to_shoulder_distance:
                #         right_arm_below = True

                # if left_arm_above or (not right_wrist and not right_shoulder):
                #     self.robot_arm.goal_x = 0.10
                #     self.robot_arm.goal_y = 0.20
                # elif left_arm_below or (not right_wrist and not right_shoulder):
                #     self.robot_arm.goal_x = -0.10
                #     self.robot_arm.goal_y = 0.20
                # elif right_arm_above or (not left_wrist and not left_shoulder):
                #     self.robot_arm.goal_x = 0
                #     self.robot_arm.goal_y = 0.10
                # elif right_arm_below or (not left_wrist and not left_shoulder):
                #     self.robot_arm.goal_x = 0
                #     self.robot_arm.goal_y = 0.30
                # elif left_arm_above and right_arm_above:
                #     self.robot_arm.goal_x = 0
                #     self.robot_arm.goal_y = 0.20


                annotated_frame = results[0].plot()
                cv2.imshow('Results', annotated_frame)
                cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args=args)
    node = YOLO_Pose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()


