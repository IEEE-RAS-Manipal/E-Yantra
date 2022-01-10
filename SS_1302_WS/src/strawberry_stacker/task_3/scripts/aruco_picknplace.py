#!/usr/bin/env python3

"""
SS_1302 submission for Task 3.1

This module controls the drone using OFFBOARD mode control.
It picks up a box and drops it at another location.

classes:
StateMonitor    Monitors state and pose of drone
DroneControl    Controls the drone using OFFBOARD
"""

import sys
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yaml import cyaml
import rospy
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from gazebo_ros_link_attacher.srv import Gripper
import cv2
from cv2 import aruco


class StateMonitor:
    """
    StateMonitor Monitors drone state and pose

    This class contains callback functions of the subscribers of this node that track the state and
    pose of the drone during operation.
    """

    def __init__(self) -> None:
        """
        __init__ Initialises class attributes

        This method initialises the class attributes of the StateMonitor class, namely the state
        of the drone, and the drone's current pose during operation.
        """
        self.current_vel = Twist()  # Velocity of the drone
        self.current_state = State()  # State object for tracking drone state
        self.goal_pose = PoseStamped()  # Goal pose for the drone
        self.event_log = threading.Event()  # Thread tracker for the data stream
        # State of the gripper (setting dummy initial value)
        self.gripper_state = None
        self.aruco_check = False
        self.xcor = None
        self.ycor = None
        self.bridge = CvBridge()

        rospy.logwarn("State Monitor active!")

    def state_callback(self, state: State) -> None:
        """
        state_callback Callback function for state_subscriber

        This is the callback function for the state_subscriber Subscriber for the /mavros/state
        topic.

        :param state: The current state/mode of the drone.
        :type state: State
        """
        self.current_state = state  # Recording current state of the drone

    def pose_callback(self, curr_pose: PoseStamped) -> None:
        """
        pose_callback Callback function for pose_subscriber

        This is the callback function for the pose_subscriber Subscriber for the /mavros/
        local_position/pose topic.

        :param curr_pose: The current pose of the drone.
        :type curr_pose: PoseStamped
        """
        self.curr_x = curr_pose.pose.position.x
        self.curr_y = curr_pose.pose.position.y
        self.curr_z = curr_pose.pose.position.z

        def is_near(current: float, goal: float) -> bool:
            """
            is_near Setpoint tolerance checker

            This nested function checks whether the drone has reached the setpoint within the
            accepted tolerance range.

            :param pose: The pose variable being checked (X, Y or Z)
            :type pose: str
            :param current: Current pose variable
            :type current: float
            :param goal: Goal pose variable
            :type goal: float
            :return: Confirmation whether the drone is within tolerance or not
            :rtype: bool
            """
            return abs(current - goal) < 0.1

        if (
            is_near(curr_pose.pose.position.x, self.goal_pose.pose.position.x)
            and is_near(curr_pose.pose.position.y, self.goal_pose.pose.position.y)
            and is_near(curr_pose.pose.position.z, self.goal_pose.pose.position.z)
        ):
            drone_control.reached = True
            self.event_log.set()

    def gripper_callback(self, grip_detect: String) -> None:
        """
        gripper_callback Callback function for gripper_subscriber

        Callback function for gripper_subscriber to check if the gripper is in contact with
        something that can be gripped

        :param grip_detect: Detects if gripper position is valid for gripping
        :type grip_detect: String
        """
        self.gripper_state = grip_detect

    def detect_ArUco_callback(self, img):
        '''
        Detecting the Arucos in the image and extracting ID and coordinate values.
        '''
        try:
            # Converting the image to OpenCV standard image
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        Detected_ArUco_markers = {}
        corners = []
        ids = []
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (marker_corner, marker_id) in zip(corners, ids):
                Detected_ArUco_markers[marker_id] = marker_corner

            for key, _ in Detected_ArUco_markers.items():
                x_0, y_0 = map(int, Detected_ArUco_markers[key][0][0])
                x_2, y_2 = map(int, Detected_ArUco_markers[key][0][2])
            # calculating the centre point of aruco
            self.c_x = int((x_0+x_2)/2)
            self.c_y = int((y_0+y_2)/2)

            if self.c_x in range(100, 300) and self.c_y in range(100, 300):
                print("within the range!")
                self.aruco_check = True

    def pos_adjust(self):
        """
        This function adjusts the position of the drone using the quadrant method
        """
        self.target_lock = False
        self.xcor = self.curr_x
        self.ycor = self.curr_y
        v = 0.5
        while self.aruco_check and not self.target_lock:
            if self.c_x not in range(0, 400) or self.c_y not in range(0, 400):
                self.aruco_check = False
                rospy.logwarn(" Box out of sight!!!")
            # Narrowing the range
            if self.c_x in range(190, 210) and self.c_y in range(190, 210):
                self.target_lock = True
                rospy.loginfo("Target position locked!")
            # First quadrant (top-right)
            elif self.c_x > 200 and self.c_y < 200:
                print("1st QUADRANT")
                state_monitor.current_vel.linear.x = v
                state_monitor.current_vel.linear.y = v
            # Second quadrant (top-left)
            elif self.c_x < 200 and self.c_y < 200:
                print("2st QUADRANT")
                state_monitor.current_vel.linear.x = -v
                state_monitor.current_vel.linear.y = v
            # Third quadrant (bottom-left)
            elif self.c_x < 200 and self.c_y > 200:
                print("3rd QUADRANT")
                state_monitor.current_vel.linear.x = -v
                state_monitor.current_vel.linear.y = -v
            # Fourth quadrant (bottom-right)
            elif self.c_x > 200 and self.c_y > 200:
                print("4th QUADRANT")
                state_monitor.current_vel.linear.x = v
                state_monitor.current_vel.linear.y = -v


class DroneControl:
    """
    DroneControl Controls the drone

    This class contains functions to control the drone by sending data to the FCU.
    """

    def __init__(self) -> None:
        """
        __init__ Initialises class attributes

        Initiliases the class attributes for the DroneControl class, namely the setmode services
        and the data stream to the drone.
        """
        # Initialising proxy services for mode setting for the drone
        rospy.loginfo("Initialising Services...")
        # Drone Arming Service
        self.arm_service = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("mavros/cmd/arming")
        # Drone Flight Mode Service
        self.set_mode_service = rospy.ServiceProxy("mavros/set_mode", SetMode)
        rospy.wait_for_service("mavros/set_mode")
        # Drone Gripper Service
        self.gripper_service = rospy.ServiceProxy("/activate_gripper", Gripper)
        rospy.wait_for_service("/activate_gripper")
        rospy.loginfo("Services initialised.")
        self.grip_status = False
        self.stream_var = 0
        self.count = 0

        # Setting up data stream to the drone in a separate thread
        try:
            rospy.loginfo("Setting up Data Stream to Drone...")
            self.data_stream = threading.Thread(target=self.drone_data_stream)
            self.data_stream.start()
            rospy.loginfo("Data Stream operational.")
        except threading.ThreadError:
            rospy.signal_shutdown("Unable to start Data Stream! Restart!")
            sys.exit()
        # Checks if the drone has reached a goal setpoint
        self.reached = False

        rospy.logwarn("Drone Control Active!")

    def drone_arming(self) -> None:
        """
        drone_arming Arms the drone

        Arms the drone for flight control
        """
        try:
            self.arm_service(True)
        except rospy.ServiceException as exception:
            rospy.logerr(f"Service arming call failed: {exception}")

    def drone_set_mode(self, mode: str):
        """
        drone_set_mode Set mode for drone

        Sets the mode of operation for the drone based on input

        :param mode: The mode to engage on the drone
        :type mode: str
        """
        try:
            self.set_mode_service(custom_mode=mode)
        except rospy.ServiceException:
            rospy.logerr(
                f"service set_mode call failed. \n {mode} Mode not set. Check GPS is enabled"
            )

    def drone_startup(self) -> None:
        """
        drone_startup Activates drone for flight

        This method arms the drone for flight by clubbing drone_arming and drone_set_mode
        ("OFFBOARD") together for code conciseness.
        """
        # Arming the drone
        rospy.loginfo("Arming Drone...")
        while not state_monitor.current_state.armed:
            drone_control.drone_arming()
            RATE.sleep()
        rospy.logwarn("Drone Armed!")

        # Switching state to OFFBOARD
        rospy.loginfo("Turning on OFFBOARD Mode...")
        while not state_monitor.current_state.mode == "OFFBOARD":
            drone_control.drone_set_mode("OFFBOARD")
            RATE.sleep()
        rospy.logwarn("OFFBOARD Mode Activated!")

    def drone_shutdown(self) -> None:
        """
        drone_shutdown Deactivates drone after flight

        This method lands and disarms the drone by clubbing drone_set_mode("AUTO.LAND") and
        disarming for code conciseness.
        """
        # Land the drone
        rospy.loginfo("Landing Drone...")
        while not state_monitor.current_state.mode == "AUTO.LAND":
            drone_control.drone_set_mode("AUTO.LAND")
            RATE.sleep()
        # Waiting for drone to disarm after landing
        while state_monitor.current_state.armed:
            RATE.sleep()
        rospy.logwarn("Drone Landed and Disarmed!")

    def drone_data_stream(self):
        """
        drone_data_stream Establishes a data stream

        Continuous data stream to the drone for setpoint transmission
        """

        while not rospy.is_shutdown():
            try:
                if self.stream_var == 1:
                    try:
                        velocity_publisher.publish(state_monitor.current_vel)
                        RATE.sleep()
                    except ROSInterruptException:
                        rospy.loginfo("Velocity Stream terminated.")
                else:
                    try:
                        position_publisher.publish(state_monitor.goal_pose)
                        RATE.sleep()
                    except ROSInterruptException:
                        rospy.loginfo("Data Stream terminated.")
            except ROSInterruptException:
                rospy.loginfo("Data Stream terminated.")

    def drone_set_goal(self, setpt: list, vel: float = 2):
        """
        drone_set_goal Sets goal setpoint for drone

        Internal pipeline function that feeds the setpoints to be set for the next traversal of
        the drone. This goal will be checked with the current pose to determine whether it has
        reached.

        :param setpt: The goal setpoint to be sent to the drone.
        :type setpt: list
        :param vel: Velocity of the drone, defaults to 5
        :type vel: float, optional
        """

        self.reached = False
        # Setting goal position coordinates
        state_monitor.goal_pose.pose.position.x = setpt[0]
        state_monitor.goal_pose.pose.position.y = setpt[1]
        state_monitor.goal_pose.pose.position.z = setpt[2]
        # Setting drone speed
        state_monitor.current_vel.linear.x = vel
        state_monitor.current_vel.linear.y = 0
        state_monitor.current_vel.linear.z = 0

        if setpt == [9, 0, 3] and self.count == 0:
            drone_control.stream_var = 1
            self.count = self.count + 1
        # Wating for the drone to reach the setpoint
        while not self.reached and not state_monitor.aruco_check:
            RATE.sleep()

        rospy.loginfo(f"Reached setpoint: \033[96m{setpt}\033[0m")

    def drone_gripper_attach(self, activation: bool) -> None:
        """
        drone_gripper_attach Drone Gripper Control

        Controls the gripper of the drone for picking and placing of objects with the drone.

        :param activation: Command sent to activate or deactivate the gripper
        :type activation: bool
        """
        try:
            while not self.grip_status:
                if activation:
                    self.grip_status = self.gripper_service(activation).result
                else:
                    self.gripper_service(activation)
                    self.grip_status = True
        except rospy.ServiceException:
            pass


if __name__ == "__main__":
    try:
        rospy.init_node("pick_n_place", anonymous=True)  # Initialising node
        RATE = rospy.Rate(20.0)  # Setting rate of transmission
        rospy.logwarn("Node Started!")

        rospy.loginfo("\033[93mPerforming pre-flight startup...\033[0m")
        # Initialising state monitor
        state_monitor = StateMonitor()

        # Defining the setpoints for travel
        setpoints = [
            [0, 0, 3],
            [9, 0, 3],
            [0, 0, 3],
            [0, 0, 0]  # Last setpoint
        ]

        # Initialising publishers and subscribers
        # Position publisher
        position_publisher = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        # Velocity publisher
        velocity_publisher = rospy.Publisher(
            "mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10
        )
        rospy.loginfo("Publishers initialised!")

        # State subscriber
        state_subscriber = rospy.Subscriber(
            "mavros/state", State, state_monitor.state_callback, queue_size=10
        )
        #   Pose subscriber
        pose_subscriber = rospy.Subscriber(
            "mavros/local_position/pose",
            PoseStamped,
            state_monitor.pose_callback,
            queue_size=10,
        )
        # Drone Gripper Subscriber
        gripper_subscriber = rospy.Subscriber(
            "/gripper_check", String, state_monitor.gripper_callback
        )

        # Camera input subscriber
        cam_subscriber = rospy.Subscriber(
            "/eDrone/camera/image_raw", Image, state_monitor.detect_ArUco_callback)

        rospy.loginfo("Subscribers initialised!")

        # Initialising drone control
        drone_control = DroneControl()

        rospy.loginfo("\033[92mReady for task! Commencing flight!\033[0m")
        # Activating drone for flight
        drone_control.drone_startup()

        # Performing flight operations
        for i in setpoints[:]:
            # Sending flight setpoints
            rospy.loginfo(f"New setpoint: \033[96m{i}\033[0m")
            drone_control.drone_set_goal(i)

            # Beginning picking procedure
            if state_monitor.aruco_check:
                state_monitor.pos_adjust()
                drone_control.stream_var = 0  # Restarting setpoints datastream
                cx = state_monitor.xcor
                cy = state_monitor.ycor
                drone_control.drone_set_goal([cx, cy, 3])
                rospy.loginfo(
                    "\033[93mCommencing pickup of package...\033[0m")
                # Using approach points for precision
                rospy.loginfo(
                    f"Using approach setpoint: \033[96m[{cx}, {cy}, 2]\033[0m")
                drone_control.drone_set_goal([cx, cy, 2], 0.01)
                rospy.loginfo(
                    f"Using approach setpoint: \033[96m[{cx}, {cy}, 0.1]\033[0m")
                drone_control.drone_set_goal([cx, cy, 0.1], 0.01)
                # Landing the drone
                # drone_control.drone_shutdown()

                # Performing gripping procedure
                # Checking if the gripper is in position
                while not state_monitor.gripper_state:
                    print(state_monitor.gripper_state)
                    RATE.sleep()
                rospy.loginfo("Attempting to grip...")
                # Activating the gripper
                drone_control.drone_gripper_attach(True)
                rospy.loginfo(
                    "\033[92mPackage picked! Proceeding to dropoff point!\033[0m"
                )
                state_monitor.aruco_check = False

                # Taking off
                # drone_control.drone_startup()
                rospy.loginfo(f"New setpoint: \033[96m[{cx}, {cy}, 3]\033[0m")
                drone_control.drone_set_goal([cx, cy, 3])
                rospy.loginfo(f"New setpoint: \033[96m[9, 0, 3]\033[0m")
                drone_control.drone_set_goal([9, 0, 3])

            # Beginning placing procedure
            # Enters the loop if the goal is 9,0,3 AND if the drone is holding the box
            if drone_control.grip_status:
                rospy.loginfo(
                    "\033[93mCommencing placement of package...\033[0m")
                # Using approach point for precision
                rospy.loginfo(
                    "Using approach setpoint: \033[96m[9, 0, 0.2]\033[0m")
                drone_control.drone_set_goal([9, 0, 0.2], 0.01)
                # Landing the drone
                drone_control.drone_shutdown()

                # Deactivating the gripper
                rospy.loginfo("Deactivating gripper...")
                drone_control.drone_gripper_attach(False)
                rospy.loginfo(
                    "\033[92mPackage placed! Proceeding to home position!\033[0m"
                )

                # Taking off
                drone_control.drone_startup()
                rospy.loginfo("New setpoint: \033[96m[9, 0, 3]\033[0m")
                drone_control.drone_set_goal([9, 0, 3])

        # Begin final post-flight landing procedure
        drone_control.drone_shutdown()

        # Ending Operations
        rospy.loginfo("\033[92mTask complete! Shutting down Node!\033[0m")
        rospy.signal_shutdown("Node Ended!")
        rospy.logwarn("Node Shutdown Succesful!")

    except ROSInterruptException:
        rospy.logwarn("Node Terminated by User!")
