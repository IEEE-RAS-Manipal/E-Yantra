#!/usr/bin/env python3

"""
SS_1302 submission for Task 4

This module controls multiple drones using OFFBOARD mode control. Each drone detects an ArUco
marker pasted on a box on the ground using OpenCV, while travelling between defined setpoints, and
controls the drone to pick and place that box in a truck depending on the colour of the box.

classes:
Drone           Contains all logic for controlling a single drone entity
DroneMonitor    Monitors drone parameters for controlling the drone
DroneControl    Contains various control commands for the drone to run
"""

# Importing essential packages and libraries
# Module control and core Python librarios
from concurrent.futures import thread
import sys  # Main system thread control
import threading  # Multithreading
from math import exp  # Misc. math functions
from typing import List  # Misc. math functions


# Core ROS
import rospy  # ROS Python libraries
from rospy.exceptions import ROSInterruptException, ROSException  # ROS Node execution

# Drone control
from std_msgs.msg import String  # Gripper status
from geometry_msgs.msg import PoseStamped, Twist  # Pose and velocity data
from mavros_msgs.msg import State  # Current drone state
from mavros_msgs.srv import CommandBool, SetMode  # Setting drone flight mode
from gazebo_ros_link_attacher.srv import Gripper  # Gripper commands
import tf.transformations  # for quaternion

# ArUco detection
import cv2  # Main OpenCV libraries
import numpy  # Misc. operations if required
from cv2 import aruco  # ArUco marker detection
from sensor_msgs.msg import Image  # Image data from drone
from cv_bridge import (
    CvBridge,
    CvBridgeError,
)  # Conversion between ROS and OpenCV Images


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
        rospy.loginfo("Subscribers initialised.")

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

            q = [0, 0, 0, 0]
            self.current_pose = curr_pose
            '''
            q[0] = self.current_pose.pose.orientation.w  # Quaternion
            q[1] = self.current_pose.pose.orientation.x
            q[2] = self.current_pose.pose.orientation.y
            q[3] = self.current_pose.pose.orientation.z
            euler = tf.transformations.euler_from_quaternion(q)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            print("roll= " + str(roll) + "; pitch= " +
                  str(pitch) + "; yaw= "+str(yaw))
                  '''

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
                    # print(detected_markers[key][0][:])
                    self.aruco_centre[0] = [
                        int((x_0 + x_2) / 2),
                        int((y_0 + y_2) / 2),
                    ]

                    # if self.current_pose.pose.orientation.
                    self.aruco_check = True  # Announcing detection of marker

    class DroneControl:
        """
        DroneControl Controls the drone

        This class contains low level functions to control the drone by sending data to the FCU.
        """

        def __init__(self, drone_id) -> None:
            """
            __init__ Initialises class attributes

            Initiliases the class attributes for the DroneControl class, namely all the ROS
            services required for drone control.

            :param drone_id: The drone number to initialise.
            :type drone_id: int
            """
            rospy.loginfo("Initialising drone control.")

            # Initialising drone monitor instance
            self.drone_monitor = Drone.DroneMonitor()
            self.drone_id = drone_id  # Drone number
            # Stream switch flag to switch between position and velocity setpoint tramsission
            self.stream_switch = False

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
                except rospy.ServiceException as exception:
                    rospy.logerr(f"Service arming call failed: {exception}")
            rospy.sleep(3)
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

        def drone_set_goal(self, goal_pose: list, override: bool = False, relative: bool = False, tolerance: float = 0.2) -> None:
            """
            drone_set_goal Sets goal setpoint for drone

            Feeds the setpoints to be set for the next traversal of the drone. The goal setpoint
            will be checked with the current state to determine whether the drone has reached. The
            drone will also check for markers along the way, although this criterion can be
            overriden to make the drone ignore markers.

            :param goal_pose: The goal setpoint to be sent to the drone.
            :type goal_pose: list
            :param override: Overrides the marker detection criteria.
            :type override: bool
            """
            # Resetting flags
            reached = False  # Checks whether drone has reach goal setpoint
            self.stream_switch = False
            # Calculating the goal coordinates relative to the drone's position

            if not relative:
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
                return abs(current - goal) < tolerance  # Tolerance radius = 0.2m

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

        def drone_package_pick(self, package_pos: list) -> None:
            """
            drone_package_pick Picks up a package lying at the designated location

            This function performs approach and pickup of a detected package, which the last known
            position of the drone when the package was detected passed in arguments. This is used
            along with the real-time position of the ArUco marker to adjust the velocity of the
            drone on its approach. The function exits once the package has been successfully
            picked.

            :param package_pos: Last-known position of detected package, to be used as reference.
            :type package_pos: list
            """
            package_pos.append(3)

            # Velocity of the drone to be tweaked during approach
            vel = [0.0, 0.0, 0.0]
            rospy.loginfo(
                f"\033[93mDrone #{self.drone_id+1} commencing pickup of package near \033[96m{package_pos}...\033[0m"
            )
            rospy.loginfo(
                f"\033[93mDrone #{self.drone_id+1} going back...\033[0m"
            )
            self.drone_set_goal(package_pos, True, True, 0.3)

            self.stream_switch = True  # Switch to velocity setpoint transmission

            while self.drone_monitor.current_state.armed:
                # Real-time position of ArUco marker
                [aruco_cx, aruco_cy] = self.drone_monitor.aruco_centre[0][:]

                # Tweaking velocity of the drone using the exp(0.4x-3) function
                vel[0] = exp(
                    0.5
                    * abs(
                        package_pos[0] -
                        self.drone_monitor.current_pose.pose.position.x
                    )
                    - 2
                )
                vel[1] = exp(
                    0.5
                    * abs(
                        package_pos[1] -
                        self.drone_monitor.current_pose.pose.position.y
                    )
                    - 2
                )
                # Velocity along the Z axis follows exp(0.4z-1) curve
                vel[2] = exp(
                    (0.4
                     * self.drone_monitor.current_pose.pose.position.z) - 1)

                # If ArUco is in this range, begin descending
                quad_x = 200
                quad_y = 200

                if (aruco_cx in range(170, 230)) and (aruco_cy in range(170, 230)) and self.drone_monitor.current_pose.pose.position.z > 2:
                    self.drone_monitor.goal_vel.linear.z = -vel[2]

                if self.drone_monitor.current_pose.pose.position.z < 2:
                    quad_x = 200
                    quad_y = 325
                    self.drone_monitor.goal_vel.linear.z = -exp(
                        (0.4
                         * self.drone_monitor.current_pose.pose.position.z) - 3)

                    if self.drone_monitor.current_pose.pose.position.z <= 0.2:
                        self.drone_monitor.goal_vel.linear.x = 0
                        self.drone_monitor.goal_vel.linear.y = 0

                        '''self.drone_monitor.goal_vel.linear.z = -exp(
                                (0.4
                                 * self.drone_monitor.current_pose.pose.position.z) - 2)
                        '''
                        self.drone_shutdown()

                if self.drone_monitor.current_pose.pose.position.z in range(1, 2):
                    self.drone_monitor.goal_vel.linear.z = 0
                    # Checking the coordinates again and locking the position
                    if (aruco_cx in range(195, 205)) and (aruco_cy in range(195, 205)):
                        package_pos[0] = self.drone_monitor.current_pose.pose.position.x + 0.2
                        # adding offset
                        package_pos[1] = self.drone_monitor.current_pose.pose.position.y + 0.2
                        self.drone_monitor.goal_vel.linear.x = 0
                        self.drone_monitor.goal_vel.linear.y = 0
                        print(" Insideeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
                        package_pos[2] = 2

                        self.stream_switch = False  # beginning to transmit setpoints
                        self.drone_set_goal(package_pos, True, True, 0.1)
                        self.stream_switch = True
                        # '''

                        # package_pos[2] = 0.5  # approaching...
                        #self.drone_set_goal(package_pos, True, True)
                        # package_pos[2] = 0  # approaching...
                        # RATE.sleep()
                        #self.drone_set_goal(package_pos, True, True)

                # Positional adjustment based on relative quadrant-based approach
                # First quadrant (top-right)
                if aruco_cx > quad_x and aruco_cy < quad_y:
                    self.drone_monitor.goal_vel.linear.x = vel[0]
                    self.drone_monitor.goal_vel.linear.y = vel[1]
                # Second quadrant (top-left)
                elif aruco_cx < quad_x and aruco_cy < quad_y:
                    self.drone_monitor.goal_vel.linear.x = -vel[0]
                    self.drone_monitor.goal_vel.linear.y = vel[1]
                # Third quadrant (bottom-left)
                elif aruco_cx < quad_x and aruco_cy > quad_y:
                    self.drone_monitor.goal_vel.linear.x = -vel[0]
                    self.drone_monitor.goal_vel.linear.y = -vel[1]
                # Fourth quadrant (bottom-right)
                elif aruco_cx > quad_x and aruco_cy > quad_y:
                    self.drone_monitor.goal_vel.linear.x = vel[0]
                    self.drone_monitor.goal_vel.linear.y = -vel[1]

                '''
                if ((aruco_cx in range(180, 220)) and (aruco_cy in range(180, 220)) and self.drone_monitor.current_pose.pose.position.z > 1.0) or ((aruco_cx in range(190, 210)) and (aruco_cy in range(310, 330)) and self.drone_monitor.current_pose.pose.position.z <= 1.0):
                    package_pos[0] = self.drone_monitor.current_pose.pose.position.x
                    package_pos[1] = self.drone_monitor.current_pose.pose.position.y
                    self.drone_monitor.goal_vel.linear.x = 0
                    self.drone_monitor.goal_vel.linear.y = 0
                    self.drone_monitor.goal_vel.linear.z = -vel[2]
                else:
                    self.drone_monitor.goal_vel.linear.z = -0.05

                if self.drone_monitor.current_pose.pose.position.z > 1.0:
                    # These variables represent the centre of the quadrant system
                    quad_x = 200
                    quad_y = 200
                else:
                    quad_x = 200
                    quad_y = 320

                # Positional adjustment based on relative quadrant-based approach
                # First quadrant (top-right)
                if aruco_cx > quad_x and aruco_cy < quad_y:
                    self.drone_monitor.goal_vel.linear.x = vel[0]
                    self.drone_monitor.goal_vel.linear.y = vel[1]
                # Second quadrant (top-left)
                elif aruco_cx < quad_x and aruco_cy < quad_y:
                    self.drone_monitor.goal_vel.linear.x = -vel[0]
                    self.drone_monitor.goal_vel.linear.y = vel[1]
                # Third quadrant (bottom-left)
                elif aruco_cx < quad_x and aruco_cy > quad_y:
                    self.drone_monitor.goal_vel.linear.x = -vel[0]
                    self.drone_monitor.goal_vel.linear.y = -vel[1]
                # Fourth quadrant (bottom-right)
                elif aruco_cx > quad_x and aruco_cy > quad_y:
                    self.drone_monitor.goal_vel.linear.x = vel[0]
                    self.drone_monitor.goal_vel.linear.y = -vel[1]

                # Slowing down when close to the ground
                if self.drone_monitor.current_pose.pose.position.z <= 0.2:
                    # Manual delay for ensuring smooth transition
                    # rospy.sleep(2)
                    self.drone_monitor.goal_vel.linear.x = 0
                    self.drone_monitor.goal_vel.linear.y = 0
                    self.drone_monitor.goal_vel.linear.z = 0
                    self.drone_shutdown()
                '''

                RATE.sleep()

            self.stream_switch = False  # Switch back to normal setpoint tranmission

            # Performing gripping procedure
            # Checking if the gripper is in position
            while not self.drone_monitor.gripper_state:
                print(self.drone_monitor.gripper_state)
                RATE.sleep()
            rospy.loginfo("Attempting to grip...")
            # Activating the gripper
            self.drone_gripper_attach(True)
            rospy.loginfo(
                f"\033[92mDrone #{self.drone_id+1} package picked! Proceeding to dropoff point!\033[0m"
            )

            # Taking off from location
            self.drone_startup()
            package_pos[2] = 3
            self.drone_set_goal(package_pos, True, True)

        def drone_package_place(self, place_pos: list) -> None:
            """
            drone_package_place Places a package at the designated location

            This function places the package being carried by the drone at the designated
            location. It is assumed that a package has been picked by the drone prior to executing
            this function.

            :param place_pos: The location to place the package.
            :type place_pos: list
            """
            rospy.loginfo(
                f"\033[93mDrone #{self.drone_id+1} commencing placement of package at\033[96m{place_pos}...\033[0m"
            )
            self.drone_set_goal(place_pos, True)

            # Landing the drone
            self.drone_shutdown()

            # Deactivating the gripper
            rospy.loginfo("Deactivating gripper...")
            self.drone_gripper_attach(False)
            rospy.loginfo("Gripper deactivated.")
            rospy.loginfo(
                f"\033[92mDrone #{self.drone_id+1}Package placed!\033[0m")

            # Taking off
            self.drone_startup()
            self.drone_set_goal(place_pos, True)
            self.drone_monitor.aruco_check = False
            self.drone_monitor.aruco_centre[0] = [0, 0]

        def drone_gripper_attach(self, activation: bool) -> None:
            """
            drone_gripper_attach Drone Gripper Control

            Controls the gripper of the drone for picking and placing of objects with the drone.

            :param activation: Command sent to activate or deactivate the gripper
            :type activation: bool
            """
            grip_status = False
            try:
                while not grip_status:
                    if activation:
                        grip_status = self.gripper_service(activation).result
                    else:
                        self.gripper_service(activation)
                        grip_status = True
                grip_status = False
            except rospy.ServiceException:
                pass

        def drone_row_patrol(self, rownum: int):
            z = 4
            div = 5
            val = True  # Used as the 'Override' variable
            start = [[1, 1, z], [1, 5, z], [1, 9, z], [1, 13, z], [1, 17, z], [1, 21, z], [1, 25, z], [1, 29, z], [
                1, 33, z], [1, 37, z], [1, 41, z], [1, 45, z], [1, 49, z], [1, 53, z], [1, 57, z], [1, 61, z]]

            row_coord = start[rownum-1]
            while not self.drone_monitor.aruco_check and row_coord[0] <= 60:
                self.drone_set_goal(row_coord, val)
                row_coord[0] = row_coord[0] + (60/div)
                val = False  # Override variable becomes False once the drone enters the row


def drone1ops() -> None:
    """
    drone1ops Multithreaded function for Drone #1 operations

    # 1.
    This function runs in a separate thread from the main module thread, and performs the operations required of Drone
    """
    try:
        drone1 = Drone(0)

        # Defining the setpoints for travel
        setpoints = [
            [-1, 1, 3],
            [2, 17, 4],
            [15, 17, 4],
            [30, 17, 4],
            [45, 17, 4],
            [60, 17, 4],
            [-1, 1, 4],
        ]
        drone1.drone_control.drone_set_goal(setpoints[0], override=True)
        drone1.drone_control.drone_row_patrol(5)
        '''
        drone1.drone_control.drone_set_goal(setpoints[1], override=True)
        drone1.drone_control.drone_set_goal(setpoints[2])
        '''
        if drone1.drone_control.drone_monitor.gripper_state:  # truck
            drone1.drone_control.drone_set_goal(
                [15.55, 0, 4], True)  # turning point
            drone1.drone_control.drone_set_goal([14.7, -3.94, 3], True)
            drone1.drone_control.drone_package_place(
                [14.7, -4.94, 3])  # Placing package
        rospy.loginfo("Back to scanning...")
        drone1.drone_control.drone_set_goal(
            [15.55, 1, 3], True)  # turning point
        drone1.drone_control.drone_set_goal([1, 22, 3], True)
        drone1.drone_control.drone_set_goal([2, 24, 3], True)
        print("gonna start patrolling now")
        drone1.drone_control.drone_set_goal([20, 24, 3])
        if drone1.drone_control.drone_monitor.gripper_state:
            drone1.drone_control.drone_set_goal([15.55, 1, 3], True)
            drone1.drone_control.drone_set_goal([15.55, -4.94, 3], True)
            drone1.drone_control.drone_package_place(
                [15.55, -4.94, 3])  # Placing package

        drone1.drone_control.drone_set_goal(setpoints[5], True)

        rospy.loginfo("\033[92mDrone #1 completed flight!\033[0m")
    except ROSInterruptException:
        pass


def drone2ops() -> None:
    """
    drone2ops Multithreaded function for Drone #2 operations

    # 2.
    This function runs in a separate thread from the main module thread, and performs the operations required of Drone
    """
    try:
        drone2 = Drone(1)
        # Defining the setpoints for travel
        setpoints = [
            [-1, 61, 3],
            [1, 49, 4],
            [15, 49, 4],
            [30, 49, 4],
            [45, 49, 4],
            [60, 49, 4],
            [0, 0, 4],
        ]

        drone2.drone_control.drone_set_goal(setpoints[0], override=True)
        drone2.drone_control.drone_set_goal(setpoints[1], override=True)
        drone2.drone_control.drone_set_goal(setpoints[2])

        if drone2.drone_control.drone_monitor.gripper_state:
            drone2.drone_control.drone_set_goal(
                [57.35, 62, 4], True)  # Turning point
            drone2.drone_control.drone_set_goal([57.35, 64.75, 3], True)
            drone2.drone_control.drone_package_place(
                [57.35, 64.75, 2])  # Placing package
        rospy.loginfo("Back to scanning...")
        drone2.drone_control.drone_set_goal(
            [57.35, 62, 4], True)  # Turning point
        drone2.drone_control.drone_set_goal([1, 29, 4], True)
        drone2.drone_control.drone_set_goal([20, 29, 4])
        if drone2.drone_control.drone_monitor.gripper_state:
            drone2.drone_control.drone_set_goal(
                [57.35, 62, 3], True)  # Turning point
            drone2.drone_control.drone_set_goal([58.2, 64.75, 3], True)
            drone2.drone_control.drone_package_place(
                [58.2, 64.75, 2])
        drone2.drone_control.drone_set_goal(
            [57.35, 62, 3], True)  # Turning point
        drone2.drone_control.drone_set_goal(setpoints[0], True)

        rospy.loginfo("\033[92mDrone #2 completed flight!\033[0m")
    except ROSInterruptException:
        pass


def truck_inventory(choice: int) -> List[float]:
    """
    truck_inventory Update truck inventory

    Based on the choice of truck, this function calculates the cell that is currently free for a
    package to be placed. The cell coordinates are returned for the drone to place the package at.

    :param choice: The choice of red or blue truck.
    :type choice: int
    :return: The coordinates of the free cell of the selected truck.
    :rtype: List[float]
    """
    # Local cell counter variables for operations
    i = TRUCK[choice][1][0]
    j = TRUCK[choice][1][1]

    # Calculating the cell coordinates [x+i, y+j, z]
    cell = [TRUCK[choice][0][0] + i * 0.85,
            TRUCK[choice][0][1] + j * 1.23, 1.7]

    # Incrementing local counter and updating global counter
    j += 1
    if j >= 2:
        j = 0
        i += 1
    TRUCK[choice][1][0] = i
    TRUCK[choice][1][1] = j

    return cell


if __name__ == "__main__":
    try:
        rospy.init_node("ss1302_task4", anonymous=True)  # Initialising node
        RATE = rospy.Rate(10)  # Setting rate of transmission
        rospy.logwarn("Node Started!")
        # Specify the initial positions of the drones
        DRONE_HOME = [[-1, 1, 0], [-1, 61, 0]]
        rospy.loginfo("\033[93mPerforming pre-flight startup...\033[0m")
        try:
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
