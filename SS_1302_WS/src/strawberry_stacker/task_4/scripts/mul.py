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
import sys  # Main system thread control
import threading  # Multithreading
from math import exp  # Misc. math functions

# Core ROS
import rospy  # ROS Python libraries
from rospy.exceptions import ROSInterruptException  # ROS Node execution

# Drone control
from std_msgs.msg import String  # Gripper status
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
REACHED = False


class Drone:
    """
    Drone Contains all logic for controlling a single drone

    This class contains higher logic for controlling a single drone entity. It lists DroneMonitor
    and DroneControl as child classes to hold respective monitor data and control commands for a
    single drone entity.
    """

    def __init__(self, drone_id) -> None:
        """
        __init__ Initialises class attributes

        This method initialises the class attributes of the Drone class, namely initialising the
        DroneMonitor and DroneControl classes for a single drone entity.
        """
        rospy.logwarn(f"Starting up Drone #{drone_id+1}...")
        self.drone_id = drone_id

        # Initialising child classes
        self.drone_monitor = self.DroneMonitor(drone_id)
        self.drone_control = self.DroneControl(drone_id)

        # Initialising publishers and subscribers
        rospy.loginfo("Initialising publishers...")
        # Position publisher
        self.position_publisher = rospy.Publisher(
            f"edrone{drone_id}/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10,
        )
        # Velocity publisher
        self.velocity_publisher = rospy.Publisher(
            f"edrone{drone_id}/mavros/setpoint_velocity/cmd_vel_unstamped",
            Twist,
            queue_size=10,
        )
        rospy.loginfo("Publishers initialised.")
        rospy.loginfo("Initialising subscribers...")
        # State subscriber
        self.state_subscriber = rospy.Subscriber(
            f"edrone{drone_id}/mavros/state",
            State,
            self.drone_monitor.state_callback,
            queue_size=10,
        )
        # Pose subscriber
        self.pose_subscriber = rospy.Subscriber(
            f"edrone{drone_id}/mavros/local_position/pose",
            PoseStamped,
            self.drone_monitor.pose_callback,
            queue_size=10,
        )
        # Drone Gripper Subscriber
        self.gripper_subscriber = rospy.Subscriber(
            f"edrone{drone_id}/gripper_check",
            String,
            self.drone_monitor.gripper_callback,
        )
        # ArUco detection camera subscriber
        self.cam_subscriber = rospy.Subscriber(
            f"iris_{drone_id}/camera/image_raw",
            Image,
            self.drone_monitor.aruco_callback,
        )
        rospy.loginfo("Subscribers initialised.")

        # Setting up data stream to the drone in a separate thread
        try:
            # Stream switch flag to switch to velocity setpoint tramsission
            self.stream_switch = False
            rospy.loginfo("Initialising data stream...")
            self.data_stream = threading.Thread(target=self.drone_data_stream)
            self.data_stream.start()
            rospy.loginfo("Data stream initialised.")
        except threading.ThreadError:
            rospy.signal_shutdown("Unable to start Data Stream! Restart!")
            sys.exit()

        # Starting up drone
        self.drone_startup()

    def drone_data_stream(self) -> None:
        """
        drone_data_stream Establishes a data stream

        Continuous data stream to the drone for setpoint transmission. Transmits position
        setpoints by default, but can also switch to velocity setpoints if required.
        """

        while not rospy.is_shutdown():
            try:
                if not self.stream_switch:  # Position setpoints
                    self.position_publisher.publish(
                        self.drone_monitor.goal_pose)
                else:  # Velocity setpoints
                    self.velocity_publisher.publish(
                        self.drone_monitor.goal_vel)
                RATE.sleep()
            except ROSInterruptException:
                rospy.loginfo("Data Stream terminated.")

    def drone_startup(self) -> None:
        """
        drone_startup Activates drone for flight

        This method arms the drone and sets OFFBOARD mode for flight.
        """
        # Arming the drone
        rospy.loginfo("Arming Drone...")
        while not self.drone_monitor.current_state.armed:
            try:
                self.drone_control.arm_service(True)
            except rospy.ServiceException as exception:
                rospy.logerr(f"Service arming call failed: {exception}")
            RATE.sleep()
        rospy.loginfo("Drone armed.")

        # Switching state to OFFBOARD
        rospy.loginfo("Switching to OFFBOARD mode...")
        while not self.drone_monitor.current_state.mode == "OFFBOARD":
            self.drone_control.drone_flight_mode("OFFBOARD")
            RATE.sleep()
        rospy.loginfo("OFFBOARD mode activated.")
        rospy.logwarn(f"Drone #{self.drone_id+1} ready for flight.")

    def drone_shutdown(self) -> None:
        """
        drone_shutdown Deactivates drone after flight

        This method lands and disarms the drone by clubbing drone_flight_mode("AUTO.LAND") and
        disarming for code conciseness.
        """
        # Land the drone
        rospy.loginfo("Landing drone...")
        while not self.drone_monitor.current_state.mode == "AUTO.LAND":
            self.drone_control.drone_flight_mode("AUTO.LAND")
            RATE.sleep()
        # Waiting for drone to disarm after landing
        while self.drone_monitor.current_state.armed:
            RATE.sleep()
        rospy.loginfo("Drone landed and disarmed.")

    class DroneMonitor:
        """
        DroneMonitor Monitors drone state and pose

        This class contains callback functions of the subscribers of this node that track the
        state and pose of the drone during operation.
        """

        def __init__(self, drone_id) -> None:
            """
            __init__ Initialises class attributes

            This method initialises the class attributes of the DroneMonitor class, namely the state
            of the drone, and the drone's current pose during operation.
            """
            rospy.loginfo("Initialising drone monitor...")

            # Drone monitoring attributes
            self.goal_vel = Twist()  # Velocity of the drone
            self.current_state = State()  # State object for tracking drone state
            self.current_pose = PoseStamped()  # Pose of the drone
            self.goal_pose = PoseStamped()  # Goal pose for the drone
            self.event_log = (
                threading.Event()
            )  # Thread tracker for the drone's data stream
            # State of the gripper (dummy initial value)
            self.gripper_state = None
            # self.reached = False  # Checks if the drone has reached a goal setpoint

            # ArUco detection attributes
            self.bridge = (
                CvBridge()
            )  # Conversion between ROS Image Messages and OpenCV Images
            # Coordinates of centre of detected aruco markers
            self.aruco_centre = [[]]
            self.aruco_check = False  # Marker detection confirmation flag
            self.callback_delay = 0  # Manual delay timer

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
            /mavros/local_position/pose topic.

            :param curr_pose: The current pose of the drone.
            :type curr_pose: PoseStamped
            """
            global REACHED
            # Recording current pose for usage in aruco_callback
            self.current_pose = curr_pose

            # Nested function for tolerance radius checking
            def is_near(current: float, goal: float) -> bool:
                """
                is_near Setpoint tolerance checker

                This nested function inside pose_callback checks whether the drone has reached the
                setpoint within the accepted tolerance range.

                :param pose: The pose variable being checked (X, Y or Z)
                :type pose: str
                :param current: Current pose variable
                :type current: float
                :param goal: Goal pose variable
                :type goal: float
                :return: Confirmation whether the drone is within tolerance or not
                :rtype: bool
                """
                return abs(current - goal) < 0.2  # Tolerance radius = 0.2m

            if (
                is_near(curr_pose.pose.position.x,
                        self.goal_pose.pose.position.x)
                and is_near(curr_pose.pose.position.y, self.goal_pose.pose.position.y)
                and is_near(curr_pose.pose.position.z, self.goal_pose.pose.position.z)
            ):
                REACHED = True
                self.event_log.set()

        def gripper_callback(self, grip_detect: String) -> None:
            """
            gripper_callback Callback function for gripper_subscriber

            This is the callback function for gripper_subscriber Subscriber for the /gripper_check
            topic.

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
                    self.aruco_centre[key - 1] = [
                        int((x_0 + x_2) / 2),
                        int((y_0 + y_2) / 2),
                    ]
                    self.aruco_check = True  # Announcing detection of marker

    class DroneControl:
        """
        DroneControl Controls the drone

        This class contains functions to control the drone by sending data to the FCU.
        """

        def __init__(self, drone_id) -> None:
            """
            __init__ Initialises class attributes

            Initiliases the class attributes for the DroneControl class, namely the setmode
            services
            and the data stream to the drone.
            """
            rospy.loginfo("Initialising drone control.")

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

        def drone_flight_mode(self, mode: str):
            """
            drone_flight_mode Set flight mode for drone

            Sets the mode of operation for the drone based on input.

            :param mode: The mode to engage on the drone
            :type mode: str
            """
            try:
                self.set_mode_service(custom_mode=mode)
            except rospy.ServiceException:
                rospy.logerr(
                    f"service set_mode call failed. \n {mode} Mode not set. Check GPS is enabled"
                )

        def drone_set_goal(self, goal_pose: list, override: bool = False) -> None:
            """
            drone_set_goal Sets goal setpoint for drone

            Feeds the setpoints to be set for the next traversal of the drone. This goal will be
            checked with the current state to determine whether it has reached. It will also check for
            markers along the way (this can be overriden).

            :param goal_pose: The goal setpoint to be sent to the drone.
            :type goal_pose: list
            :param override: Overrides the marker detection criteria.
            :type override: bool
            """
            global REACHED
            # Resetting flags
            REACHED = False
            self.stream_switch = False

            rospy.loginfo(
                f"New setpoint: \033[96m[{goal_pose[0]}, {goal_pose[1]}, {goal_pose[2]}]\033[0m"
            )
            # Setting goal position coordinates
            # will this work? i want it to say drone0 or drone1 depending on the value of n
            drone1.drone_monitor.goal_pose.pose.position.x = goal_pose[0]
            drone1.drone_monitor.goal_pose.pose.position.y = goal_pose[1]
            drone1.drone_monitor.goal_pose.pose.position.z = goal_pose[2]

            # Wating for the drone to reach the setpoint or detect a marker
            while not REACHED:
                if drone1.drone_monitor.aruco_check and not override:
                    break
                RATE.sleep()

            rospy.loginfo(f"Reached setpoint: \033[96m{goal_pose}\033[0m")

            if drone1.drone_monitor.aruco_check and not override:
                package_pos = [
                    drone1.drone_monitor.current_pose.pose.position.x,
                    drone1.drone_monitor.current_pose.pose.position.y,
                ]
                rospy.loginfo(
                    f"\033[92mArUco marker detected at \033[96m{package_pos}\033[0m"
                )
                self.drone_package_pick(package_pos)

            # Resetting flags
            REACHED = False
            self.stream_switch = False

        def drone_package_pick(self, package_pos: list) -> None:
            """
            drone_package_pick Picks up a package lying at the designated location

            This function performs approach and pickup of a detected package, which the last known
            position of the drone when the package was detected passed in arguments. This is used
            along with the real-time position of the ArUco marker to adjust the velocity of the drone
            on its approach. The function exits once the package has been successfully picked.

            :param package_pos: Last-known position of detected package, to be used as reference.
            :type package_pos: list
            """

            # Velocity of the drone to be tweaked during approach
            vel = [0.0, 0.0, 0.0]
            rospy.loginfo("\033[93mCommencing pickup of package...\033[0m")

            drone1.stream_switch = True  # Switch to velocity setpoint transmission

            rospy.loginfo("Performing adjustments...")
            while drone1.drone_monitor.current_state.armed:
                # Real-time position of ArUco marker
                [aruco_cx, aruco_cy] = drone1.drone_monitor.aruco_centre[0][:]

                # Tweaking velocity of the drone using the exp(0.4x-3) function
                vel[0] = exp(
                    0.4
                    * abs(
                        package_pos[0] -
                        drone1.drone_monitor.current_pose.pose.position.x
                    )
                    - 3
                )
                vel[1] = exp(
                    0.4
                    * abs(
                        package_pos[1] -
                        drone1.drone_monitor.current_pose.pose.position.y
                    )
                    - 3
                )

                # If ArUco is in this range, begin descending
                if (aruco_cx in range(150, 170)) and (aruco_cy in range(190, 210)):
                    package_pos[0] = drone1.drone_monitor.current_pose.pose.position.x
                    package_pos[1] = drone1.drone_monitor.current_pose.pose.position.y
                    drone1.drone_monitor.goal_vel.linear.z = -0.4

                # Positional adjustment based on relative quadrant-based approach
                # First quadrant (top-right)
                if aruco_cx > 160 and aruco_cy < 200:
                    drone1.drone_monitor.goal_vel.linear.x = vel[0]
                    drone1.drone_monitor.goal_vel.linear.y = vel[1]
                # Second quadrant (top-left)
                elif aruco_cx < 160 and aruco_cy < 200:
                    drone1.drone_monitor.goal_vel.linear.x = -vel[0]
                    drone1.drone_monitor.goal_vel.linear.y = vel[1]
                # Third quadrant (bottom-left)
                elif aruco_cx < 160 and aruco_cy > 200:
                    drone1.drone_monitor.goal_vel.linear.x = -vel[0]
                    drone1.drone_monitor.goal_vel.linear.y = -vel[1]
                # Fourth quadrant (bottom-right)
                elif aruco_cx > 160 and aruco_cy > 200:
                    drone1.drone_monitor.goal_vel.linear.x = vel[0]
                    drone1.drone_monitor.goal_vel.linear.y = -vel[1]

                # Slowing down when close to the ground
                if drone1.drone_monitor.current_pose.pose.position.z < -0.2:
                    drone1.drone_monitor.goal_vel.linear.z = -0.15
                if drone1.drone_monitor.current_pose.pose.position.z < -0.3:
                    # Manual delay for ensuring smooth transition
                    rospy.sleep(2)
                    drone1.drone_shutdown()
                RATE.sleep()

            drone1.drone_control.stream_switch = (
                False  # Switch back to normal setpoint tranmission
            )

            # Performing gripping procedure
            # Checking if the gripper is in position
            while not drone1.drone_monitor.gripper_state:
                print(drone1.drone_monitor.gripper_state)
                RATE.sleep()
            rospy.loginfo("Attempting to grip...")
            # Activating the gripper
            drone1.drone_control.drone_gripper_attach(True)
            rospy.loginfo(
                "\033[92mPackage picked! Proceeding to dropoff point!\033[0m")

            # Taking off from location
            drone1.drone_control.drone_startup()
            package_pos.append(3)
            drone1.drone_control.drone_set_goal(package_pos, True)

        def drone_package_place(self, place_pos: list) -> None:
            """
            drone_package_place Places a package at the designated location

            This function places the package being carried by the drone at the designated location. It
            is assumed that a package has been picked by the drone prior to executing this function.

            :param place_pos: The location to place the package.
            :type place_pos: list
            """
            rospy.loginfo(
                f"\033[93mCommencing placing of package at\033[96m{place_pos}\033[0m"
            )
            self.drone_control.drone_set_goal(place_pos, True)

            # Landing the drone
            self.drone_control.drone_shutdown()

            # Deactivating the gripper
            rospy.loginfo("Deactivating gripper...")
            self.drone_control.drone_gripper_attach(False)
            rospy.loginfo(
                "\033[92mPackage placed! Proceeding on original path!\033[0m")

            # Taking off
            self.drone_control.drone_startup()
            self.drone_control.drone_set_goal(place_pos, True)

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


if __name__ == "__main__":
    try:
        rospy.init_node("ss1302_task4", anonymous=True)  # Initialising node
        RATE = rospy.Rate(10)  # Setting rate of transmission
        rospy.logwarn("Node Started!")

        rospy.loginfo("\033[93mPerforming pre-flight startup...\033[0m")
        #drone0 = Drone(0)
        drone1 = Drone(1)
        rospy.loginfo("\033[92mReady for task! Commencing flight!\033[0m")

        # # Defining the setpoints for travel
        setpoints = [
            [0, 0, 3],
            [20, -14, 3],
            [20, -12, 0.5],
            [57.5, 3.75, 5],
            [57.5, 3.75, 1.7]
            #    [0, 0, 3],
        ]  # Box placing setpoint

        # Performing flight operations
        # Sending flight setpoints
        drone1.drone_control.drone_set_goal(setpoints[0])
        print("aaaaaaaaaaaaaaaaaaaaaa")
        drone1.drone_control.drone_set_goal(setpoints[1])
        drone1.drone_control.drone_set_goal(setpoints[2])

        # # Will execute only if something has been picked, for debugging purposes
        # if self.drone_monitor.gripper_state:
        #     self.drone_control.drone_package_place(setpoints[1])  # Placing package
        # rospy.loginfo("Proceeding to home position.")
        # rospy.sleep(1)
        # self.drone_control.drone_set_goal(setpoints[2], True)

        # # Begin final post-flight landing procedure
        # self.drone_control.drone_shutdown()

        # Ending Operations
        rospy.loginfo("\033[92mTask complete! Shutting down Node!\033[0m")
        rospy.signal_shutdown("Node Ended!")
        rospy.logwarn("Node Shutdown Succesful!")

    except ROSInterruptException:
        rospy.logwarn("Node Terminated by User!")
