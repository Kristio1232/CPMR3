###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def set_gripper(base, position):
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Close the gripper with position increments
    print("Performing gripper test in position...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger.value = position
    print(f"Going to position {position}")
    base.SendGripperCommand(gripper_command)


def get_gripper(base):
    gripper_request = Base_pb2.GripperRequest()
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
    if len (gripper_measure.finger):
        print(f"Current position is : {gripper_measure.finger[0].value}")
        return gripper_measure.finger[0].value
    return None

 
def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def example_angular_action_movement(base, angles=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    print(actuator_count.count)
    if actuator_count.count != len(angles):
        print(f"bad lengths {actuator_count.count} {len(angles)}")
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = angles[joint_id]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def example_cartesian_action_movement(base, base_cyclic):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x          # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y - 0.1    # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z - 0.2    # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True

        success &= example_move_to_home_position(base)
        set_gripper(base, 1.0)
        for i in range(10):
            print(get_gripper(base))
            time.sleep(0.2)
        set_gripper(base, 0)
        for i in range(10):
            print(get_gripper(base))
            time.sleep(0.2)
#        success &= example_cartesian_action_movement(base, base_cyclic)
        success &= example_angular_action_movement(base, [0,0,0,0,0,0])
        success &= example_angular_action_movement(base, [154,0,0,0,0,0])
        success &= example_angular_action_movement(base, [-154,0,0,0,0,0])
        success &= example_angular_action_movement(base, [0,0,0,0,0,0])
        success &= example_angular_action_movement(base, [0,-45,0,0,0,0])
        success &= example_angular_action_movement(base, [0,45,0,0,0,0])
        success &= example_angular_action_movement(base, [0,0,0,0,0,0])
        success &= example_angular_action_movement(base, [0,0,45,0,0,0])
        success &= example_angular_action_movement(base, [0,0,-45,0,0,0])
        success &= example_angular_action_movement(base, [0,0,0,0,0,0])
        success &= example_angular_action_movement(base, [0,0,0,45,0,0])
        success &= example_angular_action_movement(base, [0,0,0,-45,0,0])
        success &= example_angular_action_movement(base, [0,0,0,0,0,0])
        success &= example_angular_action_movement(base, [0,0,0,0,45,0])
        success &= example_angular_action_movement(base, [0,0,0,0,-45,0])
        success &= example_angular_action_movement(base, [0,0,0,0,0,0])
        success &= example_angular_action_movement(base, [0,0,0,0,0,45])
        success &= example_angular_action_movement(base, [0,0,0,0,0,-45])
        success &= example_angular_action_movement(base, [45,-45,0,0,0,0])
        input("Press return to grab block")
        set_gripper(base, 1.0)
        time.sleep(2)
        print(get_gripper(base))
        success &= example_angular_action_movement(base, [0,-45,45,0,0,0])
        set_gripper(base, 0.0)
        time.sleep(2)
        success &= example_move_to_home_position(base)

        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())



