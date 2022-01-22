#! /usr/bin/env python
'''
Library of UR5-specific functions for MoveIt! and RViz Motion Planning.
Contains:
1) MoveIt! Parameters and Controllers for controlling the arm
    a) Go to a specified pose
    b) Go to specified join angles
    c) Play a saved trojectory from a file
2) RViz Planning Scene controls for adding, attaching, detaching and removing objects from the Planning Scene
'''
import rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, actionlib, rospkg
import yaml, sys, tf
from std_srvs.srv import Empty

class Ur5Moveit:
    '''
    Class for the UR5 Arm.
    '''

    # Constructor
    def __init__(self, arg_robot_name):
        '''
        Constructor containing all essential MoveIt and RViz assets.
        '''
        # Initialise node
        rospy.init_node('node_moveit_%s' %arg_robot_name, anonymous=True)
        
        # Transform Listener for box TF detection
        self.tf_listener = tf.TransformListener()

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description",
                                                      ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns + "/robot_description",
                                                          ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + '/execute_trajectory',
                                                                        moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._touch_links = self._robot.get_link_names(
            group=self._planning_group)
        self._box_name = ''

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):
        '''
        Goes to a specified pose and orientation.

        Parameters:
            arg_pose (Pose object): The pose and orientation to execute planning towards.
        
        Returns:
            flag_plan (bool): Confirmation whether the planning and execution was successful or not.
        '''

        # Get current pose values
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Set final pose target with given pose value
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Get final joints values calculated
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        # Verify planning/executing success
        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def hard_go_to_pose(self, arg_pose, arg_max_attempts):
        '''
        Similar to go_to_pose, but repeatedly tries to find a solution.

        Parameters:
            arg_pose (Pose object): The pose and orientation to execute planning towards.
            arg_max_attempts (int): The maximum number of attempts for the planner. 
        '''
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()


    def set_joint_angles(self, arg_list_joint_angles):
        '''
        Goes to specified joint angles.
        
        Parameters:
            arg_list_joint_angles (float[]): A list of joint angles in radians to plan towards.
        
        Returns:
            flag_plan (bool): Confirmation whether the planning and execution was successful or not.
        '''

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

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        '''
        Similar to set_joint_angles, but repeatedly tries to find a solution.

        Parameters:
            arg_list_joint_angles (float[]): List of joint angles to plan towards.
            arg_max_attempts (int): The maximum number of attempts for the planner.
        '''

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''
        Plays a saved trajectory from a file.

        Parameters:
            arg_file_path (str): The file path of the .yaml file with the saved trajectory.
            arg_file_name (str): The name of the .yaml file.
        '''
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        '''
        Similar to moveit_play_planned_path_from_file but tries repeatedly for a solution.

        Parameters:
            arg_file_path (str): The file path of the .yaml file with the saved trajectory.
            arg_file_name (str): The name of the .yaml file.
            arg_max_attempts (int): The maximum number of attempts for the planner.
        '''
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True

    # Adds box to world
    def add_box(self, box_name, box_length, box_pose):
        '''
        Adds a box to the RViz planning scene.
        
        Parameters:
            box_name (str): The name to be assigned to the box.
            box_length (float): The size of the box.
            box_pose (PoseStamped object): The pose and orientation of the box.
        '''
        self._scene.add_box(box_name,
                            box_pose,
                            size=(box_length, box_length, box_length))

    def attach_box(self, box_name):
        '''
        Attaches the specified object(box) to the robot hand.

        Parameters:
            box_name (str): The name of the box in the RViz Planning Scene te be attached.
        '''
        self._scene.attach_box(self._eef_link,
                               box_name,
                               touch_links=self._touch_links)

    def detach_box(self, box_name):
        '''
        Detaches the specified object(box) from the robot hand.

        Parameters:
            box_name (str): The name of the box in the RViz Planning Scene te be detached.
        '''
        self._scene.remove_attached_object(self._eef_link,
                                           name=box_name)

    def remove_box(self, box_name):
        '''
        Removes the specified object(box) from the RViz Planning Scene.

        Parameters:
            box_name (str): The name of the box to be removed.
        '''
        self._scene.remove_world_object(box_name)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
