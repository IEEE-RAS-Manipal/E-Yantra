#! /usr/bin/env python
'''
Controls the UR5 near the bins to pick up boxes from the conveyor.
'''

import rospy, geometry_msgs.msg, os, threading
from hrwros_gazebo.msg import LogicalCameraImage
from math import radians
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import String
from lib_task4 import Ur5Moveit

Package_Pos = 0  # Stores the conveyor position of the detected box
Package_Name = ''  # String to store models' names from camera_callback()
Package_Colours = [] # String list to store packages colours from package_callback()
Ready_Flag = False  # Start-stop variable

def env_data():
    '''
    Data of all environment-specific parameters:
    1. Vacuum Gripper Width
    2. Box Size
    3. Home Position Joint angles for the UR5

    Returns:
        All environment-specific data.
    '''
    box_length = 0.15  # Length of the box
    vacuum_gripper_width = 0.117  # Vacuum Gripper Width
    home_joint_angles = [radians(0),
                         radians(-120),
                         radians(-85),
                         radians(-65),
                         radians(90),
                         radians(0)]

    # Return data when called
    return [box_length,
            vacuum_gripper_width, home_joint_angles]


def smart_stop():
    '''
    Multithreaded function for conveyor start-stop
    '''
    global Package_Name, Package_Pos, Ready_Flag

    power_flag = False # Conveyor Signal variable

    while (True):
        if ('packagen' in Package_Name) and (abs(Package_Pos) < 0.4):
            if(power_flag): os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 0"')
            Ready_Flag = True
            power_flag = False
        elif(not power_flag):
            os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 100"')
            power_flag = True


def camera_callback(msg_camera):
    '''
    Callback function for Conveyor Logical Camera Subscriber

    Parameters:
        msg_camera (LogicalCameraImage): Data about all the objects detected by the Logical Camera.
    '''
    global Package_Name, Package_Pos

    if msg_camera.models != []:
        if len(msg_camera.models) == 1:
            Package_Name = msg_camera.models[0].type
            Package_Pos = msg_camera.models[0].pose.position.y
        else:
            Package_Name = msg_camera.models[1].type
            Package_Pos = msg_camera.models[1].pose.position.y

def package_callback(msg_package_list):
    '''
    Callback function for the package colour decoder Subscriber.

    Parameters:
        msg_package_list (str): CSV-formatted list of all the package colours in order.
    '''
    global Package_Colours, sub

    temp_var = msg_package_list.data.split(',') # Converting the CSV

    if(len(temp_var)==9 and 'NA' not in temp_var):
        rospy.logwarn("Configuration stored:")
        rospy.logwarn(temp_var)
        sub.unregister() # Once data has been retrieved once it is not required anymore
        Package_Colours = temp_var
    

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
    if (override != []):
        trans = override[0]
        rot = override[1]

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
        bax_name (str): The name of the box.
        box_length (float): The size of the box.
        vacuum_gripper_width (float): The width of the vacuum gripper.
    '''
    # Offset for end effector placement
    delta = vacuum_gripper_width + (box_length / 2)

    # Obtaining the TF transform of the box
    (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                            "/logical_camera_2_%s_frame" %box_name,
                                                            rospy.Time(0))
    # Execute pick operation
    box_pose = pose_set(box_trans, box_rot)  # Collating pose values
    box_pose.pose.position.z = box_pose.pose.position.z + delta  # Adding Z Offset
    ur5.hard_go_to_pose(box_pose, 5) # Executing pick travel
    # Activate Vacuum Gripper
    os.system(
        'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_2 "activate_vacuum_gripper: true"\n')
    # Add the box to the planning scene
    # box_pose.pose.position.z = box_pose.pose.position.z - delta  # Removing Z Offset
    # ur5.add_box(box_name, box_length, box_pose)
    # ur5.attach_box(box_name)
    # Travelling back to home
    ur5.hard_set_joint_angles(env_values[2],5)
    os.system('rosservice call /eyrc/vb/conveyor/set_power "power: 100"')
    # Log the operation
    rospy.logwarn(
        "Package '%s' picked!" %box_name)

def bin_plan(box_name, bin_name):
    '''
    Place-planning for the bins.

    Parameters:
        box_name (str): The name of the box
        bin_name (str): The colour of the bin.
    '''
    colour_code = '' # Terminal Colour Code

    # Travel to bin
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path + 'ur5_2/', 'home_to_%s.yaml' %bin_name[0], 5)
    # Deactivate the Gripper
    os.system(
        'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_2 "activate_vacuum_gripper: false"\n')
    
    # Log the operation
    if(bin_name=='red'): colour_code = '91'
    elif(bin_name=='yellow'): colour_code = '93'
    elif(bin_name=='green'): colour_code = '92'
    rospy.loginfo(
        '\033[{}m {} package {} placed! \033[0m'.format(colour_code, bin_name, box_name))
    
    # Travel to home
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path + 'ur5_2/', '%s_to_home.yaml' %bin_name[0], 5)
    
    # Remove the box from the planning scene


def subscriber_init():
    '''
    Definitions and setups of all Subscribers.
    '''
    global sub
    # Subscriber for Conveyor Logical Camera
    rospy.Subscriber('/eyrc/vb/logical_camera_2',
                    LogicalCameraImage,
                    camera_callback)

    # Subscriber for Shelf Camera for decoding package colours
    # This is only required to get data once
    sub = rospy.Subscriber('/package_colour', 
                            String,
                            package_callback)


def controller():
    '''
    Executes the main operations.
    '''
    global Package_Name, Package_Colours, Ready_Flag

    # Go to home position
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path + 'ur5_2/', 'zero_to_home.yaml', 5)

    # Execute planning
    while (not rospy.is_shutdown()):
        try:
            if len(Package_Colours)==0:
                t.join()
                quit()
            if ('packagen' in Package_Name) and Ready_Flag:  # When box detected
                box_plan(Package_Name, env_values[0], env_values[1])  # Execute pick operation
                bin_plan(Package_Name, Package_Colours[0])  # Execute place operation
                Package_Colours.pop(0) # Removing item from list
                Ready_Flag = False # Signalling conveyor

        except ROSInterruptException:
            pass

if __name__ == '__main__':
    '''
    Controls overall execution.
    '''
    rospy.sleep(10)

    ur5 = Ur5Moveit("ur5_2")

    # Start the separate conveyor control thread
    t = threading.Thread(target=smart_stop)
    t.start()

    # Obtain prerequisite data
    env_values = env_data()

    # Start the Subscribers
    subscriber_init()

    # Start execution
    controller()