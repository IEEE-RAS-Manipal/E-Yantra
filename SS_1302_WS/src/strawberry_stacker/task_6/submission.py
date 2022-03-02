#!/usr/bin/env python3

"""
SS_1302 submission for Task 6

This module controls multiple drones using OFFBOARD mode control. Each drone detects an ArUco
marker pasted on a box on the ground using OpenCV, while travelling between defined setpoints, and
controls the drone to pick and place that box in a truck depending on the colour of the box.

classes:
Drone           Contains all logic for controlling a single drone entity
DroneMonitor    Monitors drone parameters for controlling the drone
DroneControl    Contains various control commands for the drone to run
"""

# Importing essential packages and libraries
# Module control and core Python libraries
from pickletools import uint8
import sys  # Main system thread control
import threading  # Multithreading
from math import exp  # Misc. math functions
from typing import List  # Misc. math functions
import time

# Core ROS
import rospy  # ROS Python libraries
from rospy.exceptions import ROSInterruptException, ROSException  # ROS Node execution

# Drone control
from std_msgs.msg import String, UInt8  # Gripper status
from geometry_msgs.msg import PoseStamped, Twist  # Pose and velocity data
from mavros_msgs.msg import State  # Current drone state
from mavros_msgs.srv import CommandBool, SetMode  # Setting drone flight mode
from gazebo_ros_link_attacher.srv import Gripper  # Gripper commands

# ArUco detection
import cv2  # Main OpenCV libraries
import numpy  # Misc. operations if required
from cv2 import aruco  # ArUco marker detection
from sensor_msgs.msg import Image  # Image data from drone
from cv_bridge import (
    CvBridge,
    CvBridgeError,
)  # Conversion between ROS and OpenCV Images


# Initialising a global dictionary to map each row with the number of boxes present
ROWLIST = {1: 0,
           2: 0,
           3: 0,
           4: 0,
           5: 0,
           6: 0,
           7: 0,
           8: 0,
           9: 0,
           10: 0,
           11: 0,
           12: 0,
           13: 0,
           14: 0,
           15: 0
           }

# Given initial cell coordinates of the trucks
TRUCK = [[[56.7, 64.6], [0, 0]],
         [[14.1, -7.8], [0, 0]]]
# Height at which the box must be dropped
STACK_HEIGHT = 1.7


class Drone:
    """
    Drone Contains all logic for controlling a single drone

    This class contains higher logic for controlling a single drone entity. It lists DroneMonitor
    and DroneControl as nested classes to hold respective monitor data and control commands for a
    single drone entity.
    """

    def __init__(self, drone_id: int) -> None:
        """
        __init__ Initialises class attributes

        This method initialises the class attributes of the Drone class, namely initialising the
        DroneMonitor and DroneControl classes for a single drone entity along with some higher
        level parameters.

        :param drone_id: The drone number to initialise.
        :type drone_id: int
        """
        rospy.logwarn(f"Booting up Drone #{drone_id+1}...")
        self.drone_id = drone_id

        # Initialising control
        self.drone_control = self.DroneControl(self.drone_id)

        # Initialising publishers and subscribers
        rospy.loginfo("Initialising publishers...")
        # Position publisher
        self.position_publisher = rospy.Publisher(
            f"edrone{self.drone_id}/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10,
        )
        # Velocity publisher
        self.velocity_publisher = rospy.Publisher(
            f"edrone{self.drone_id}/mavros/setpoint_velocity/cmd_vel_unstamped",
            Twist,
            queue_size=10,
        )
        rospy.loginfo("Publishers initialised.")
        rospy.loginfo("Initialising subscribers...")
        # State subscriber
        self.state_subscriber = rospy.Subscriber(
            f"edrone{drone_id}/mavros/state",
            State,
            self.drone_control.drone_monitor.state_callback,
            queue_size=10,
        )
        # Pose subscriber
        self.pose_subscriber = rospy.Subscriber(
            f"edrone{self.drone_id}/mavros/local_position/pose",
            PoseStamped,
            self.drone_control.drone_monitor.pose_callback,
            queue_size=10,
        )
        # Drone Gripper Subscriber
        self.gripper_subscriber = rospy.Subscriber(
            f"edrone{self.drone_id}/gripper_check",
            String,
            self.drone_control.drone_monitor.gripper_callback,
        )
        # ArUco detection camera subscriber
        self.cam_subscriber = rospy.Subscriber(
            f"edrone{self.drone_id}/camera/image_raw",
            Image,
            self.drone_control.drone_monitor.aruco_callback,
        )

        # Setting up data stream to the drone in a separate thread
        try:
            rospy.loginfo("Initialising data stream...")
            self.data_stream = threading.Thread(target=self.drone_data_stream)
            self.data_stream.start()
            rospy.loginfo("Data stream initialised.")
        except threading.ThreadError:
            rospy.signal_shutdown("Unable to start Data Stream! Restart!")
            sys.exit()

        # Starting up drone
        self.drone_control.drone_startup()

    def drone_data_stream(self) -> None:
        """
        drone_data_stream Establishes a data stream

        Continuous data stream to the drone for setpoint transmission. Transmits position
        setpoints by default, but can also switch to velocity setpoints if required.
        """

        while not rospy.is_shutdown():
            try:

                if not self.drone_control.stream_switch:  # Position setpoints
                    self.position_publisher.publish(
                        self.drone_control.drone_monitor.goal_pose
                    )
                else:  # Velocity setpoints
                    self.velocity_publisher.publish(
                        self.drone_control.drone_monitor.goal_vel
                    )
                RATE.sleep()

            except ROSInterruptException:
                rospy.loginfo(
                    f"Drone #{self.drone_id+1} data stream terminated.")
            except ROSException:
                rospy.loginfo(
                    f"Drone #{self.drone_id+1} data stream terminated.")

    class DroneMonitor:
        """
        DroneMonitor Monitors drone state and pose

        This class contains callback functions of the subscribers of this node that track the
        the drone during operation.
        """

        def __init__(self) -> None:
            """
            __init__ Initialises class attributes

            This method initialises the class attributes of the DroneMonitor class.
            """
            rospy.loginfo("Initialising drone monitor...")

            # Drone monitoring attributes
            self.current_state = State()  # Current state/flight mode of the drone
            self.current_pose = PoseStamped()  # Pose of the drone
            self.goal_pose = PoseStamped()  # Goal pose of the drone
            # State of the gripper (dummy initial value)
            self.gripper_state = None
            self.goal_vel = Twist()  # Velocity of the drone

            # ArUco detection attributes
            self.bridge = CvBridge()
            self.aruco_ID = None

            # Coordinates of centre of detected aruco markers
            self.aruco_centre = [[]]
            self.aruco_check = False  # Marker detection confirmation flag

            rospy.loginfo("Drone monitor initialised.")

        def state_callback(self, state: State) -> None:
            """
            state_callback Callback function for state_subscriber

            This is the callback function for the state_subscriber Subscriber for the
            /mavros/state topic.

            :param state: The current state/mode of the drone.
            :type state: State
            """
            self.current_state = state  # Recording current state of the drone

        def pose_callback(self, curr_pose: PoseStamped) -> None:
            """
            pose_callback Callback function for pose_subscriber

            This is the callback function for the pose_subscriber Subscriber for the
            /mavros/local_position/pose topic. The pose of the drone is used to track it's travel
            between setpoints.

            :param curr_pose: The current pose of the drone.
            :type curr_pose: PoseStamped
            """
            self.current_pose = curr_pose

        def gripper_callback(self, grip_detect: String) -> None:
            """
            gripper_callback Callback function for gripper_subscriber

            This is the callback function for gripper_subscriber Subscriber for the /gripper_check
            topic. The state of the gripper is checked before the drone is deemed capable of
            picking up a package.

            :param grip_detect: Detects if gripper position is valid for gripping
            :type grip_detect: String
            """
            self.gripper_state = grip_detect

        def aruco_callback(self, img: numpy.array) -> None:
            """
            aruco_callback Callback function for aruco_subscriber

            This is the callback function for the aruco_subscriber Subscriber for the
            /eDrone/camera/image_raw topic.

            :param img: The raw image data received from the drone camera.
            :type img: numpy.array
            """
            # Defining local variables
            detected_markers = {}  # All detected markers
            corners = []  # Detected corners of each marker
            ids = []  # Detected ID number of each marker
            greyscale_img = None  # Greyscale converted image

            # Image extraction
            try:
                # ROS Image message to standard OpenCV image conversion
                cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
                # RGB to Grayscale conversion
                greyscale_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            except CvBridgeError:
                print(CvBridgeError)

            # Detecting ArUco markers and other data
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(
                greyscale_img, aruco_dict, parameters=parameters
            )

            # Verify that at least one ArUco marker was detected
            if ids is not None:
                # Flatten the ArUco IDs list
                ids = ids.flatten()
                # Loop over the detected ArUCo corners
                for (marker_corner, marker_id) in zip(corners, ids):
                    detected_markers[marker_id] = marker_corner
                # Calculate the centre position of the detected aruco marker
                for key, _ in detected_markers.items():
                    x_0, y_0 = map(int, detected_markers[key][0][0])
                    x_2, y_2 = map(int, detected_markers[key][0][2])
                    # print(detected_markers[key])

                    self.aruco_centre[0] = [
                        int((x_0 + x_2) / 2),
                        int((y_0 + y_2) / 2),
                    ]
                    self.aruco_ID = key

                    # if self.current_pose.pose.orientation.
                    self.aruco_check = True  # Announcing detection of marker
            else:
                self.aruco_check = False

    class DroneControl:
        """
        DroneControl Controls the drone

        This class contains low level functions to control the drone by sending data to the FCU.
        """

        def __init__(self, drone_id) -> None:
            """
            __init__ Initialises class attributes for the DroneControl class, namely all the ROS
            services required for drone control.

            :param drone_id: The drone number to initialise.
            :type drone_id: int
            """
            rospy.loginfo("Initialising drone control..")

            # Initialising drone monitor instance
            self.drone_monitor = Drone.DroneMonitor()
            self.drone_id = drone_id  # Drone number
            self.done_with_row = None

            # Stream switch flag to switch between position and velocity setpoint tramsission
            self.stream_switch = False
            self.current_row = None  # The row being traversed by the drone

            # Initialising proxy services for mode setting for the drone
            rospy.loginfo("Initialising Services...")
            # Drone Arming Service
            self.arm_service = rospy.ServiceProxy(
                f"edrone{drone_id}/mavros/cmd/arming", CommandBool
            )
            rospy.wait_for_service(f"edrone{drone_id}/mavros/cmd/arming")
            # Drone Flight Mode Service
            self.set_mode_service = rospy.ServiceProxy(
                f"edrone{drone_id}/mavros/set_mode", SetMode
            )
            rospy.wait_for_service(f"edrone{drone_id}/mavros/set_mode")
            # Drone Gripper Service
            self.gripper_service = rospy.ServiceProxy(
                f"edrone{drone_id}/activate_gripper", Gripper
            )
            rospy.wait_for_service(f"edrone{drone_id}/activate_gripper")
            rospy.loginfo("Services initialised.")

            rospy.loginfo("Drone control initialised.")

        def drone_startup(self) -> None:
            """
            drone_startup Activates drone for flight

            This method arms the drone and sets OFFBOARD mode for flight.
            """
            rospy.loginfo(
                f"\033[93mStarting Drone #{self.drone_id+1}...\033[0m")

            # Arming the drone
            rospy.loginfo(f"Arming drone #{self.drone_id+1}...")
            while not self.drone_monitor.current_state.armed:
                try:
                    self.arm_service(True)
                except rospy.ServiceException as exception:
                    rospy.logerr(f"Service arming call failed: {exception}")
                RATE.sleep()
            rospy.loginfo("Drone armed.")

            # Switching state to OFFBOARD
            rospy.loginfo(
                f"Enabling OFFBOARD flight mode for Drone #{self.drone_id+1}..."
            )
            while not self.drone_monitor.current_state.mode == "OFFBOARD":
                try:
                    self.set_mode_service(custom_mode="OFFBOARD")
                except rospy.ServiceException as exception:
                    rospy.logerr(f"Service arming call failed: {exception}")
                RATE.sleep()
            rospy.loginfo("OFFBOARD flight mode activated.")

            rospy.loginfo(
                f"\033[92mDrone #{self.drone_id+1} ready for flight!\033[90m")

        def drone_shutdown(self) -> None:
            """
            drone_shutdown Deactivates drone after flight

            This method lands and disarms the drone, using the AUTO.LAND mode for the drone
            landing.
            """
            rospy.loginfo(
                f"\033[93mShutting down Drone #{self.drone_id+1}...\033[0m")

            # Land the drone
            rospy.logwarn(f"Landing Drone #{self.drone_id+1}...")
            while not self.drone_monitor.current_state.mode == "AUTO.LAND":
                try:
                    self.set_mode_service(custom_mode="AUTO.LAND")
                    RATE.sleep
                except rospy.ServiceException as exception:
                    rospy.logerr(f"Service arming call failed: {exception}")
            time.sleep(5)
            # Waiting for drone to disarm after landing
            while self.drone_monitor.current_state.armed:
                try:
                    self.arm_service(False)
                except rospy.ServiceException as exception:
                    rospy.logerr(f"Service arming call failed: {exception}")
                RATE.sleep()
            rospy.logwarn(f"Drone #{self.drone_id+1} landed and disarmed.")

            rospy.loginfo(
                f"\033[92mDrone #{self.drone_id+1} shut down!\033[90m")

        def drone_set_goal(self, goal_pose: list, override: bool = False,
                           relative: bool = False, tolerance: float = 0.2) -> None:
            """
            drone_set_goal Sets goal setpoint for drone

            Receives both absolute or relative setpoints to be set for the
            next traversal of the drone. The goal setpoint will be checked with
            the current position to determine whether the drone has reached.
            The drone will also check for boxes along the way, although this
            criterion can be overridden to make the drone ignore aruco markers.

            :param goal_pose: The goal setpoint to be sent to the drone.
            :type goal_pose: list
            :param override: Overrides the aruco marker detection criteria
            :type override: bool
            :param relative: Flag for absolute setpoints to enable conversion to absolute
            :type relative: bool
            :param tolerance: The buffer difference between the actual pose and the desired pose
            :type tolerance: float
            """
            # Resetting flags
            reached = False  # Checks whether drone has reach goal setpoint
            self.stream_switch = False
            # Calculating the goal coordinates relative to the drone's position

            if not relative:
                # Converting absolute coordinates to relative
                goal_pose = [x1-x2 for x1,
                             x2 in zip(goal_pose, DRONE_HOME[self.drone_id])]

            rospy.loginfo(
                f"Drone #{self.drone_id+1} travelling to \033[96m[{goal_pose[0]}, {goal_pose[1]}, {goal_pose[2]}]\033[0m"
            )

            # Setting goal position coordinates
            self.drone_monitor.goal_pose.pose.position.x = goal_pose[0]
            self.drone_monitor.goal_pose.pose.position.y = goal_pose[1]
            self.drone_monitor.goal_pose.pose.position.z = goal_pose[2]

            # Nested function for tolerance radius checking
            def is_near(current: float, goal: float) -> bool:
                """
                is_near Setpoint tolerance checker

                This nested function checks whether the drone has reached the goal setpoint within
                the accepted tolerance range defined within.

                :param pose: The pose variable being checked (X, Y or Z)
                :type pose: str
                :param current: Current pose variable
                :type current: float
                :param goal: Goal pose variable
                :type goal: float
                :return: Confirmation whether the drone is within tolerance or not
                :rtype: bool
                """
                return abs(current - goal) < tolerance

            # Wating for the drone to reach the setpoint or detect a marker
            while not reached:
                if self.drone_monitor.aruco_check and not override:
                    break

                if (
                    is_near(
                        self.drone_monitor.current_pose.pose.position.x, goal_pose[0]
                    )
                    and is_near(
                        self.drone_monitor.current_pose.pose.position.y, goal_pose[1]
                    )
                    and is_near(
                        self.drone_monitor.current_pose.pose.position.z, goal_pose[2]
                    )
                ):
                    reached = True
                RATE.sleep()

            rospy.loginfo(f"Drone #{self.drone_id+1} reached setpoint.")

            if self.drone_monitor.aruco_check and not override:
                package_pos = [
                    self.drone_monitor.current_pose.pose.position.x,
                    self.drone_monitor.current_pose.pose.position.y,
                ]
                rospy.logwarn(
                    f"ArUco marker detected at \033[96m{package_pos}")
                self.drone_package_pick(package_pos)

            # Resetting flags
            reached = False  # Checks whether drone has reach goal setpoint
            self.stream_switch = False

        def drone_row_to_search(self) -> int:
            """
            drone_row_to_search accesses the global row-box dictionary, scans
            through it from the most closest row to the least closest w.r.t.
            the drone's home position and returns the closest row having a box

            :return: Number of the row having box to be picked up
            :rtype: int
            """
            if self.drone_id == 0:
                for i in range(1, 15):
                    if ROWLIST[i] > 0:  # Checking if there's atleast one box in a row
                        return i
            else:
                # Drone 2 executes the below snippet
                for i in range(15, 1, -1):
                    if ROWLIST[i] > 0:
                        return i

        def drone_package_pick(self, package_pos: list) -> None:
            """
            drone_package_pick Picks up a package lying at the designated location

            This function performs approach and pickup of a detected package, which the last known
            position of the drone when the package was detected passed in arguments. This is used
            along with the real-time position of the ArUco marker to adjust the velocity of the
            drone on its approach.

            :param package_pos: Last-known (x,y) position of detected package
            :type package_pos: list
            """
            package_pos.append(3)  # setting the height for drone

            # Velocity of the drone to be tweaked during approach
            vel = [0.0, 0.0, 0.0]
            rospy.loginfo(
                f"\033[93mDrone #{self.drone_id+1} commencing pickup of package near \033[96m{package_pos}...\033[0m"
            )

            self.drone_set_goal(package_pos, True, True, 0.2)

            # Switch to velocity command transmission
            self.drone_monitor.goal_vel.linear.z = 0
            self.stream_switch = True

            while (self.drone_monitor.current_state.armed and
                   self.drone_monitor.current_pose.pose.position.z > 0.3):
                # Real-time position of ArUco marker
                [aruco_cx, aruco_cy] = self.drone_monitor.aruco_centre[0][:]

                # Using 2 sets of error-proportional-exponential curves based on drone height
                if self.drone_monitor.current_pose.pose.position.z > 2.8:
                    vel[0] = exp(
                        0.4
                        * abs(
                            package_pos[0] -
                            self.drone_monitor.current_pose.pose.position.x
                        )
                        - 1.3
                    )
                    vel[1] = exp(
                        0.4
                        * abs(
                            package_pos[1] -
                            self.drone_monitor.current_pose.pose.position.y
                        )
                        - 1.7
                    )
                else:
                    vel[0] = exp(
                        0.5
                        * abs(
                            package_pos[0] -
                            self.drone_monitor.current_pose.pose.position.x
                        )
                        - 2.7
                    )
                    vel[1] = exp(
                        0.5
                        * abs(
                            package_pos[1] -
                            self.drone_monitor.current_pose.pose.position.y
                        )
                        - 2.7
                    )

                # Velocity along the Z axis follows exp(0.8z-1.6) curve
                vel[2] = exp(
                    (0.8
                     * self.drone_monitor.current_pose.pose.position.z) - 1.6)

                # Setting the desired position of the aruco in the image
                quad_x = 200
                quad_y = 200

                # Quadrant logic is used for position correction.
                if ((aruco_cx in range(170, 230)) and (aruco_cy in range(170, 230))
                        and self.drone_monitor.current_pose.pose.position.z > 0.8):
                    self.drone_monitor.goal_vel.linear.z = -vel[2]
                    package_pos[0] = self.drone_monitor.current_pose.pose.position.x
                    package_pos[1] = self.drone_monitor.current_pose.pose.position.y

                if self.drone_monitor.current_pose.pose.position.z < 2:
                    # Changing the desired position of the box in image accouting for camera offset
                    quad_x = 198
                    quad_y = 260
                    # Slower curve for Z velocity as we approach the box
                    self.drone_monitor.goal_vel.linear.z = -exp(
                        (0.5
                         * self.drone_monitor.current_pose.pose.position.z) - 2.2)
                    # Capturing the color of the box
                    box_id = self.drone_monitor.aruco_ID

                    if self.drone_monitor.current_pose.pose.position.z < 1.05:
                        self.drone_monitor.goal_vel.linear.z = 0
                        if (((aruco_cx in range(193, 205)) and (aruco_cy in range(258, 265))
                             and self.drone_monitor.current_pose.pose.position.z > 0.8) or self.drone_monitor.current_pose.pose.position.z <= 0.4):
                            print(" Turning off X and Y velocities..")
                            self.drone_monitor.goal_vel.linear.x = 0
                            self.drone_monitor.goal_vel.linear.y = 0

                            while self.drone_monitor.current_pose.pose.position.z > 0.3:
                                self.drone_monitor.goal_vel.linear.z = -exp(
                                    (0.5
                                     * self.drone_monitor.current_pose.pose.position.z) - 1.5)

                            while not self.drone_monitor.gripper_state:
                                print(self.drone_monitor.gripper_state)
                                RATE.sleep()
                            rospy.loginfo("Attempting to grip...")
                            # Activating the gripper
                            self.drone_gripper_attach(True)

                # Dividing the image in 4 quadrants and assigning correction velocities accordingly
                if aruco_cx > quad_x:
                    self.drone_monitor.goal_vel.linear.x = vel[0]
                elif aruco_cx < quad_x:
                    self.drone_monitor.goal_vel.linear.x = -vel[0]

                if aruco_cy > quad_y:
                    self.drone_monitor.goal_vel.linear.y = -vel[1]
                elif aruco_cy < quad_y:
                    self.drone_monitor.goal_vel.linear.y = vel[1]
                RATE.sleep()

            self.stream_switch = False  # Switch back to normal setpoint tranmission

            # Taking off from location
            self.drone_startup()
            package_pos[2] = 3
            self.drone_set_goal(package_pos, True, True)
            if self.drone_monitor.gripper_state is False:
                print(" Landing failed! Attempting to re-land")
                self.drone_package_pick([package_pos[0], package_pos[1]])

            # different heights for drones to avoid collision at turning point
            if self.drone_id == 0:
                z = 3
            else:
                z = 4
            if box_id == 2:
                # turning points prevent drone-truck collision unlike flying directly
                self.drone_set_goal([15.55, 0, z], True,
                                    False, 1)
                self.drone_package_place(truck_inventory(1, self.drone_id), 2)
            elif box_id == 1:  # red-box
                self.drone_set_goal(
                    [57.35, 62, z], True, False, 1)  # Turning point
                self.drone_package_place(
                    truck_inventory(0, self.drone_id), 1)  # Placing package

        def drone_package_place(self, place_pos: list, aruco_id: int) -> None:
            """
            drone_package_place Places a package at the designated location

            This function places the package being carried by the drone at the designated
            location. It is assumed that a package has been picked by the drone prior to executing
            this function.

            :param place_pos: The location to place the package.
            :type place_pos: list
            :param aruco_id: Stores Aruco ID
            :type aruco_id : Integer
            """
            rospy.loginfo(
                f"\033[93mDrone #{self.drone_id+1} commencing placement of package at\033[96m{place_pos}...\033[0m"
            )
            self.drone_set_goal([place_pos[0], place_pos[1], 3], True)

            self.drone_set_goal(place_pos, True, False, 0.3)
            # Deactivating the gripper
            rospy.loginfo("Deactivating gripper...")
            self.drone_gripper_attach(False)
            rospy.loginfo("Gripper deactivated.")
            rospy.loginfo(
                f"\033[92mD  Drone #{self.drone_id+1}Package placed!\033[0m")

            place_pos[2] = 3
            self.drone_set_goal(place_pos, True, False)
            self.drone_monitor.aruco_check = False
            self.drone_monitor.aruco_centre[0] = [0, 0]

            # Going back to turning point
            if self.drone_id == 0:
                # drone1 flies at a height of 3m while turning
                z = 3
            else:
                z = 4
            if aruco_id == 2:
                self.drone_set_goal([15.55, 0, z], True,
                                    False, 0.5)  # turning point
            elif aruco_id == 1:
                self.drone_set_goal(
                    [57.35, 62, z], True, False, 0.5)  # Turning point
            # current pick-place finished
            self.done_with_row = True

        def drone_gripper_attach(self, activation: bool) -> None:
            """
            drone_gripper_attach Drone Gripper Control

            Controls the gripper of the drone for picking and placing of objects with the drone.

            :param activation: Command sent to activate or deactivate the gripper
            :type activation: bool
            """
            grip_status = False
            # response = None
            try:
                counter = 0
                # The counter takes the drone out of this loop if the gripper
                # doesn't work for a few seconds
                while not grip_status and counter < 1600:
                    if activation:
                        grip_status = self.gripper_service(activation).result
                        RATE.sleep
                    else:
                        i = 0
                        while i < 2:
                            # Sending detach request twice as it was failing in a few
                            # runs in the simulation - related to Gazebo-process-died error
                            self.gripper_service(activation)
                            RATE.sleep
                            i = i+1
                        grip_status = True
                    counter = counter + 1
            except rospy.ServiceException:
                pass

        def drone_row_patrol(self, rownum: int):
            """
            drone_row_patrol Updates current row for drone
            This function updates the current row to a row with detected box.
            This function divides the row in multiple segments and sends the
            setpoints to the drone for easier patrolling.

            :param rownum:row where drone needs to go next.
            :type rownum :integer
            """

            # In case the spawn_info doesn't publish anything at the beginning,
            # this loop will wait till it gets a row to patrol
            while rownum is None:
                rownum = self.drone_row_to_search()

            self.done_with_row = False
            self.current_row = rownum
            # Decrementing - updating the row - box table
            ROWLIST[self.current_row] = ROWLIST[self.current_row] - 1

            z = 3  # The height at which we want the drones to patrol
            div = 6  # Number of divisions we want in a row for better patrolling
            val = True  # Used as the 'Override' variable

            start = [[0, 1, z], [0, 5, z], [0, 9, z], [0, 13, z], [0, 17, z], [0, 21, z],
                     [0, 25, z], [0, 29, z], [0, 33, z], [
                         0, 37, z], [0, 41, z], [0, 45, z],
                     [0, 49, z], [0, 53, z], [0, 57, z], [0, 61, z]]

            row_coord = start[rownum-1]
            while row_coord[0] <= 62 and not self.done_with_row:
                print(row_coord[0])
                self.drone_set_goal(row_coord, val, False, 0.1)
                row_coord[0] = row_coord[0] + (60/div)
                val = False  # Override variable becomes False once the drone enters the row
            print("Out of row patrol!")
            self.current_row = None


class Field:
    """
    This class has info and functions related to the farm-field.
    """

    def __init__(self) -> None:
        # Row number subscriber
        self.row_subscriber = rospy.Subscriber(
            "/spawn_info", UInt8,
            self.row_callback,
            queue_size=10,
        )
        rospy.loginfo("Row-info subscriber initialised.")

    def update_rowlist(self, rno):
        """
        This function takes in the row_number from the subscriber-callback and
        increments the number of boxes in the specified row_number in our
        global dictionary

        :param rno: row number
        :type rno: int
        """
        if rno in ROWLIST.keys():
            ROWLIST[rno] += 1

    def row_callback(self, row_num: uint8) -> None:
        """
        Row callback is the callback function for the row subscriber. This function
        is responsible for maintaining a dictionary that stores the number of boxes
        in every row.
        """
        self.update_rowlist(int(row_num.data))


def truck_inventory(n: int, drone_id: int) -> List[float]:
    """
    truck_inventory Update truck inventory

    Based on the choice of truck, this function calculates the cell that is currently free for a
    package to be placed. The cell coordinates are returned for the drone to place the package at.

    :param n: The choice of red- 0 or blue-1 truck.
    :type n: int
    :return: The coordinates of the free cell of the selected truck.
    :rtype: List[float]
    """

    # n=0 for red truck  and n=1 blue truck
    global STACK_HEIGHT

    if drone_id == 0:
        TRUCK[n][1][0] = 1
        r_i = TRUCK[n][1][0]
        r_j = TRUCK[n][1][1]
        cell = [TRUCK[n][0][0]+r_i*0.85, TRUCK[n]
                [0][1]+r_j*1.23, STACK_HEIGHT]
        TRUCK[n][1][1] += 1

        if TRUCK[n][1][1] > 2:
            TRUCK[n][1][1] = 0
            STACK_HEIGHT += 1.7

    elif drone_id == 1:
        TRUCK[n][1][0] = 3
        r_i = TRUCK[n][1][0]
        r_j = TRUCK[n][1][1]
        cell = [TRUCK[n][0][0]+r_i*0.85, TRUCK[n][0][1]+r_j*1.23, STACK_HEIGHT]
        TRUCK[n][1][1] += 1

        if TRUCK[n][1][1] > 2:
            TRUCK[n][1][1] = 0
            STACK_HEIGHT += 1.7

    rospy.loginfo(
        f"Drone{drone_id + 1} is placing the box at {r_i}{r_j} i.e. {cell} ")
    return cell


def drone1ops() -> None:
    """
    drone1ops Multithreaded function for Drone #1 operations

    This function runs in a separate thread from the main module thread,
    and performs the operations required of Drone #1
    """
    try:
        drone1 = Drone(0)

        # Defining the setpoints for travel
        setpoints = [
            [-1, 1, 3]
        ]
        drone1.drone_control.drone_set_goal(setpoints[0], override=True)
        drone1.drone_control.drone_row_patrol(
            drone1.drone_control.drone_row_to_search())

        while True:
            row = drone1.drone_control.drone_row_to_search()
            if row is None:
                # When all the boxes are picked up, the value of row would be None
                # The drone should break out of this loop and go back to its home
                break
            drone1.drone_control.drone_row_patrol(row)

        drone1.drone_control.drone_set_goal(setpoints[0], override=True)
        drone1.drone_control.drone_shutdown()

    except ROSInterruptException:
        pass


def drone2ops() -> None:
    """
    drone2ops Multithreaded function for Drone #2 operations.

    This function runs in a separate thread from the main module thread,
    and performs the operations required of Drone #2
    """
    try:
        drone2 = Drone(1)
        # Defining the setpoints for travel
        setpoints = [
            [-1, 61, 3]
        ]

        drone2.drone_control.drone_set_goal(setpoints[0], override=True)
        drone2.drone_control.drone_row_patrol(
            drone2.drone_control.drone_row_to_search())

        while True:
            row = drone2.drone_control.drone_row_to_search()
            if row is None:
                break
            drone2.drone_control.drone_row_patrol(row)

        drone2.drone_control.drone_set_goal(setpoints[0], override=True)
        drone2.drone_control.drone_shutdown()

    except ROSInterruptException:
        pass


if __name__ == "__main__":
    try:
        rospy.init_node("ss1302_task4", anonymous=True)  # Initialising node
        RATE = rospy.Rate(10)  # Setting rate of transmission
        rospy.logwarn("Node Started!")
        # Specify the initial positions of the drones
        DRONE_HOME = [[-1, 1, 0], [-1, 61, 0]]

        rospy.loginfo("\033[93mPerforming pre-flight startup...\033[0m")
        try:
            field_info = Field()
            drone1thread = threading.Thread(target=drone1ops)
            drone2thread = threading.Thread(target=drone2ops)
            drone1thread.start()
            drone2thread.start()
            drone1thread.join()
            drone2thread.join()
        except threading.ThreadError:
            rospy.signal_shutdown("Unable to start Drones! Restart!")
            sys.exit()

        # Ending Operations
        rospy.loginfo("\033[92mTask complete! Shutting down Node!\033[0m")
        rospy.signal_shutdown("Node Ended!")
        rospy.logwarn("Node Shutdown Succesful!")

    except ROSInterruptException:
        rospy.logwarn("Node Terminated by User!")
