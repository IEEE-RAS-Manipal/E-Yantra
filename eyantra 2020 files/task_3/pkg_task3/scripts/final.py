#! /usr/bin/env python
'''
Controls a UR5 Robotic Arm to sort coloured boxes into bins from a conveyor.
'''

import rospy, geometry_msgs.msg, os, threading
from hrwros_gazebo.msg import LogicalCameraImage
from math import radians
from rospy.exceptions import ROSInterruptException

from lib import UR5MoveIt

Package_Pos = 0 # Stores the conveyor position of the detected box       
MasterString = '' # String to store models' names from camera_callback()
Stop_var = False # Start-stop variable

def env_data():
    '''
    Data of all environment-specific parameters:
    1. Vacuum Gripper Width
    2. Box Size

    Returns:
        All environment-specific data.
    '''
    box_length = 0.15  # Length of the box
    vacuum_gripper_width = 0.117  # Vacuum Gripper Width

    # Return data when called
    return [box_length,
            vacuum_gripper_width]

def joint_angles_data():
    '''
    Data of all joint angles required for various known positions:
    1. Home: Ready-position for picking objects off the conveyor
    2. Red Bin: Red Bin to place Red packages
    3. Green Bin: Green Bin to place Green packages
    4. Blue Bin: Blue Bin to place Blue packages
    
    Returns: 
        Joint angles for all positions when called.
    '''
    # Home/Ready angles for picking
    home_joint_angles = [radians(180),
                        radians(-75),
                        radians(110),
                        radians(-125),
                        radians(-90),
                        radians(0)]

    # Red Bin angles
    red_bin_joint_angles = [radians(65),
                           radians(-55),
                           radians(80),
                           radians(-115),
                           radians(-90),
                           radians(0)]

    # Green bin angles
    green_bin_joint_angles = [radians(0),
                             radians(-55),
                             radians(80),
                             radians(-115),
                             radians(-90),
                             radians(0)]

    # Blue bin angles
    blue_bin_joint_angles = [radians(-95),
                            radians(-55),
                            radians(80),
                            radians(-115),
                            radians(-90),
                            radians(0)]

    # Return data when called
    return [home_joint_angles, 
            red_bin_joint_angles, 
            green_bin_joint_angles,
            blue_bin_joint_angles]

def smart_stop():
    '''
    Multithreaded function for conveyor start-stop
    '''
    global MasterString, Package_Pos, Stop_var

    counter = 0

    while(True):
        if('packagen1' in MasterString or 'packagen2' in MasterString or 'packagen3' in MasterString) and (Package_Pos < 0.4):
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 0"')
            Stop_var = True
        else:
            if(counter == 0):
                os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 100"')
                counter = 1
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 40"')
                

def camera_callback(msg_camera):
    '''
    Callback function for Conveyor Logical Camera Subscriber

    Parameters:
        msg_camera (LogicalCameraImage): Data about all the objects detected by the Logical Camera.
    '''
    global MasterString, Package_Pos
    
    if(msg_camera.models != []):
        if len(msg_camera.models) == 1:
            MasterString = msg_camera.models[0].type
            Package_Pos = msg_camera.models[0].pose.position.y
        else:
            MasterString = msg_camera.models[1].type
            Package_Pos = msg_camera.models[1].pose.position.y

def pose_set(trans, rot):
    '''
    Assigns pose values w.r.t the world-frame to a PoseStamped object.

    Parameters:
        trans (float[]): Translation Values.
        rot (float[]): RPY Rotation Values.

    Returns:
        pose (geometry_msgs.msg.PoseStamped() object): The complete pose with values.
    '''
    # If you want to override any values, use this
    override = [trans, [-0.5, -0.5, 0.5, 0.5]]
    if(override != []): 
        trans = override[0]
        rot = override[1]
        print(override)

    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]
    pose.pose.orientation.x = rot[0]
    pose.pose.orientation.y = rot[1]
    pose.pose.orientation.z = rot[2]
    pose.pose.orientation.w = rot[3]

    return pose


def box_plan(box_name, box_length, vacuum_gripper_width):
    '''
    Pick-planning for the boxes.

    Parameters:
        box_name (str): The colour of the box detected.
        box_length (float): The size of the box.
        vacuum_gripper_width (float): The width of the vacuum gripper.
    '''
    # Offset for end effector placement
    delta = vacuum_gripper_width + (box_length/2)
    try:
        if(box_name == 'R'): # Red box detected
            # Obtaining the TF transform of the box
            (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world", 
                                                                "/logical_camera_2_packagen1_frame", 
                                                                rospy.Time(0))

            # Execute pick operation
            box_pose = pose_set(box_trans, box_rot) # Collating pose values
            box_pose.pose.position.z = box_pose.pose.position.z + delta # Adding Z Offset
            ur5.go_to_pose(box_pose)
            # Activate Vacuum Gripper
            os.system(
                'rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            # Add the Red box to the planning scene
            box_pose.pose.position.z = box_pose.pose.position.z - delta # Removing Z Offset
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            # Log the operation
            rospy.loginfo(
                '\033[91m' + "Red Package Picked!" + '\033[0m')
        elif(box_name == 'G'): # Green box detected
            (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world", 
                                                                "/logical_camera_2_packagen2_frame", 
                                                                rospy.Time(0))

            box_pose = pose_set(box_trans, box_rot)
            box_pose.pose.position.z = box_pose.pose.position.z + delta
            ur5.go_to_pose(box_pose)
            os.system(
                'rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            box_pose.pose.position.z = box_pose.pose.position.z - delta
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            rospy.loginfo(
                '\033[92m' + "Green Package Picked!" + '\033[0m')
        elif(box_name == 'B'): # Blue box detected
            (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world", 
                                                                "/logical_camera_2_packagen3_frame", 
                                                                rospy.Time(0))

            box_pose = pose_set(box_trans, box_rot)
            box_pose.pose.position.z = box_pose.pose.position.z + delta
            ur5.go_to_pose(box_pose)
            os.system(
                'rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            box_pose.pose.position.z = box_pose.pose.position.z - delta
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            rospy.loginfo(
                '\033[94m' + "Blue Package Picked!" + '\033[0m')
    except:
        return

def bin_plan(bin_name, bin_joint_angles):
    '''
    Place-planning for the bins.

    Parameters:
        bin_name (str): The colour of the bin.
        bin_joint_angles (float[]): The joint anglenv_values[0], env_values[1]es of the required bin.
    '''
    if(bin_name == 'R'): # Red bin
        # Set joint angles for the bin
        ur5.set_joint_angles(bin_joint_angles)
        # Deactivate the Gripper
        os.system(
            'rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        # Remove the box from the planning scene
        ur5.detach_box(bin_name)
        ur5.remove_box(bin_name)
        # Log the operation
        rospy.loginfo(
            '\033[91m' + "Red Package Placed!" + '\033[0m')
    elif(bin_name == 'G'): # Green bin
        ur5.set_joint_angles(bin_joint_angles)
        os.system(
            'rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        ur5.detach_box(bin_name)
        ur5.remove_box(bin_name)
        rospy.loginfo(
            '\033[92m' + "Green Package Placed!" + '\033[0m')
    elif(bin_name == 'B'): # Blue bin
        ur5.set_joint_angles(bin_joint_angles)
        os.system(
            'rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        ur5.detach_box(bin_name)
        ur5.remove_box(bin_name)
        rospy.loginfo(
            '\033[94m' + "Blue Package Placed!" + '\033[0m')

def subscriber_init():
    '''
    Definitions and setups of all Subscribers.
    '''
    # Subscriber for Conveyor Logical Camera
    rospy.Subscriber('/eyrc/vb/logical_camera_2',
                     LogicalCameraImage, 
                     camera_callback)

def controller():
    '''
    Executes the main operations.
    '''
    global MasterString, Stop_var

    # Go to home position
    ur5.set_joint_angles(joint_angles[0])

    # Execute planning
    while(not rospy.is_shutdown()):
        try:
            if('packagen1' in MasterString) and Stop_var: # Red box
                box_plan('R', env_values[0], env_values[1]) # Pick operation
                bin_plan('R', joint_angles[1]) # Place operation
                # ur5.set_joint_angles(joint_angles[0])
                Stop_var = False

            elif('packagen2' in MasterString) and Stop_var: # Green box
                box_plan('G', env_values[0], env_values[1])
                bin_plan('G', joint_angles[2])
                # ur5.set_joint_angles(joint_angles[0])
                Stop_var = False

            elif('packagen3' in MasterString) and Stop_var: # Blue box
                box_plan('B', env_values[0], env_values[1])
                bin_plan('B', joint_angles[3])
                # ur5.set_joint_angles(joint_angles[0]) 
                Stop_var = False
        except ROSInterruptException:
            t.join()


if __name__ == '__main__':
    '''
    Controls overall execution.
    '''
    ur5 = UR5MoveIt()

    # Start the separate conveyor control thread
    t = threading.Thread(target=smart_stop)
    t.start()

    # Obtain prerequisite data
    joint_angles = joint_angles_data()
    env_values = env_data()

    # Start the Subscribers
    subscriber_init()

    # Start execution
    controller()