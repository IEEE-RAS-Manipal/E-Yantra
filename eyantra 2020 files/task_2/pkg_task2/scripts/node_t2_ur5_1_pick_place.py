#! /usr/bin/env python
import os
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import radians


class Ur5Moveit:

    # Constructor
    def __init__(self):
        
        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)
        self._box_name = "box"
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._touch_links = self._robot.get_link_names(group=self._planning_group)

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Adds box to world
    def add_box(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.45
        box_pose.pose.position.z = 1.9
        box_pose.pose.orientation.w = 1.0
        self._scene.add_box(self._box_name, box_pose, size=(0.15, 0.15, 0.15))

    # Attaches box to robot
    def attach_box(self):
        self._scene.attach_box(self._eef_link, self._box_name, touch_links=self._touch_links)

    # Detaches box from robot
    def detach_box(self):
        self._scene.remove_attached_object(self._eef_link, name=self._box_name)

    # Removes box from world
    def remove_box(self):
        self._scene.remove_world_object(self._box_name)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    ur5 = Ur5Moveit()

    lst_joint_angles_home = [radians(0),
                             radians(0),
                             radians(0),
                             radians(0),
                             radians(0),
                             radians(0)]

    lst_joint_angles_object_pick = [radians(-65),
                                    radians(-106),
                                    radians(-0),
                                    radians(-73),
                                    radians(65),
                                    radians(0)]

    lst_joint_angles_object_waypoint = [radians(20),
                                        radians(-88),
                                        radians(35),
                                        radians(-126),
                                        radians(69),
                                        radians(0)]

    lst_joint_angles_object_place = [radians(19),
                                     radians(-140),
                                     radians(-88),
                                     radians(-38),
                                     radians(93),
                                     radians(0)]

    # Adding box to planning scene
    ur5.add_box()
    # Travelling to pick the box
    ur5.set_joint_angles(lst_joint_angles_object_pick)
    os.system(
        'rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')

    # Attaching box in planning scene
    ur5.attach_box()
    # Travelling to place the box
    ur5.set_joint_angles(lst_joint_angles_object_waypoint)
    ur5.set_joint_angles(lst_joint_angles_object_place)
    os.system(
        'rosservice call /eyrc/vb/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')

    # Removing box from planning scene
    ur5.detach_box()
    ur5.remove_box()
    # Travelling back to home position
    ur5.set_joint_angles(lst_joint_angles_home)

    del ur5

if __name__ == '__main__':
    main()
