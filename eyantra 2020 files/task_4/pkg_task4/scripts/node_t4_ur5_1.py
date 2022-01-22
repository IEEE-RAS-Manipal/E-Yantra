#! /usr/bin/env python
'''
This script controls the UR5 near the shelf for placing boxes on the conveyor.
'''

import rospy
import os
from lib_task4 import Ur5Moveit # The library for the UR5s

def main():
    '''
    Main execution
    '''
    ur5 = Ur5Moveit("ur5_1") # Initialising the UR5
    
    # Executing pick-place for 1st package
    rospy.loginfo(
            '\033[96m' + "Executing pick operation for packagen00" + '\033[0m')
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path + 'ur5_1/', 'zero_to_pkg1.yaml', 5)

    package_list = ['00', '01', '02', '10', '11', '12', '20', '21', '22']

    # Executing pick-place for the other boxes
    for i in range(1,9):
        # Activate gripper
        os.system(
        'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_1 "activate_vacuum_gripper: true"')
        # Log operation
        rospy.loginfo(
            '\033[96m' + "Executing place operation for packagen%s" %package_list[i-1] + '\033[0m')  
        # Execute place operation
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path + 'ur5_1/', 'pkg{}_to_place.yaml'.format(str(i)), 5)
        # Deactivate gripper
        os.system(
        'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_1 "activate_vacuum_gripper: false"')
        # Log next operation
        rospy.loginfo(
            '\033[96m' + "Executing pick operation for packagen%s" %package_list[i] + '\033[0m')
        # Executing pick operation for next package
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path + 'ur5_1/', 'place_to_pkg{}.yaml'.format(str(i+1)), 5)

    # Executing place for 9th package 
    os.system(
    'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_1 "activate_vacuum_gripper: true"')
    rospy.loginfo(
        '\033[96m' + "Executing place operation for packagen%s" %package_list[i-1] + '\033[0m')  
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path + 'ur5_1/', 'pkg9_to_place.yaml', 5)
    os.system(
    'rosservice call /eyrc/vb/ur5/activate_vacuum_gripper/ur5_1 "activate_vacuum_gripper: false"')
        
    # Travelling to home
    rospy.loginfo(
            '\033[96m' + "Travelling to home" + '\033[0m')
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path + 'ur5_1/', 'place_to_home.yaml', 5)

    del ur5

if __name__ == '__main__':
    rospy.sleep(10)

    main()

