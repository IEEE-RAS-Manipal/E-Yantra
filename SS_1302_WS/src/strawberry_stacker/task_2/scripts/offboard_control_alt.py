#!/usr/bin/env python3

"""
SS_1302 submission for Task 2.2

This module controls the drone using OFFBORAD mode control.

classes:
StateMonitor    Monitors state and pose of drone
DroneControl    Controls the drone using OFFBOARD
"""

import sys
import threading
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


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
        self.current_state = State()  #   State object for tracking drone state
        self.goal_pose = PoseStamped()
        self.event_log = threading.Event()

        rospy.logwarn("State Monitor Active!")

    def state_callback(self, state: State) -> None:
        """
        state_callback Callback function for state_subscriber

        This is the callback function for the state_subscriber Subscriber for the /mavros/state
        topic.

        :param state: The current state/mode of the drone.
        :type state: State
        """
        self.current_state = state  #   Recording current state of the drone

    def pose_callback(self, curr_pose: PoseStamped) -> None:
        """
        pose_callback Callback function for pose_subscriber

        This is the callback function for the pose_subscriber Subscriber for the /mavros/
        local_position/pose topic.

        :param curr_pose: The current pose of the drone.
        :type curr_pose: PoseStamped
        """

        def is_near(pose: str, current: float, goal: float) -> bool:
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
            rospy.logdebug(
                "Position %s: local: %d, target: %d, abs diff: %d",
                pose,
                current,
                goal,
                abs(current - goal),
            )
            return abs(current - goal) < 0.5

        if (
            is_near("X", curr_pose.pose.position.x, self.goal_pose.pose.position.x)
            and is_near("Y", curr_pose.pose.position.y, self.goal_pose.pose.position.y)
            and is_near("Z", curr_pose.pose.position.z, self.goal_pose.pose.position.z)
        ):
            drone_control.reached = True
            self.event_log.set()


class DroneControl:
    """
    DroneControl Controls the drone

    This class contains functions to control the drone by sending data to the FCU.
    """

    def __init__(self) -> None:
        try:
            self.data_stream = threading.Thread(target=self.drone_data_stream)
            self.data_stream.start()
        except:
            rospy.logwarn("Unable to start thread!3")
        self.reached = False
        rospy.logwarn("Drone Control Active!")

    def arm_set_mode(self) -> None:
        """
        arm_set_mode Arms the drone

        Arms the drone for flight control
        """
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service("mavros/cmd/arming")  # Waiting untill the service starts

        try:
            arm_service = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
            # Creating a proxy service for the rosservice named /mavros/cmd/arming to arm the drone
            arm_service(True)
        except rospy.ServiceException as exception:
            rospy.logerr(f"Service arming call failed: {exception}")

    def drone_set_mode(self, mode: str):
        """
        drone_set_mode Set mode for drone

        Sets the mode of operation for the drone based on input

        :param mode: The mode to engage on the drone
        :type mode: str
        """
        # Waiting untill the service starts
        rospy.wait_for_service("mavros/set_mode")
        try:
            set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
            set_mode(custom_mode=mode)
        # and print fail message on failure
        except rospy.ServiceException:
            # and print fail message on failure
            rospy.logerr(
                f"service set_mode call failed. \n {mode} Mode not set. Check GPS is enabled"
            )

    def drone_data_stream(self):
        """
        drone_data_stream Establishes a data stream

        Continuous data stream to the drone for setpoint transmission
        """

        while not rospy.is_shutdown():
            position_publisher.publish(state_monitor.goal_pose)
            RATE.sleep()

    def set_goal(self, setpt: list):
        """
        set_goal Sets goal setpoint for drone

        Internal pipeline function that feeds the setpoints to be set for the next traversal of
        the drone. This goal will be checked with the current pose to determine whether it has
        reached.

        :param setpt: The goal setpoint to be sent to the drone.
        :type setpt: list
        """
        self.reached = False
        state_monitor.goal_pose.pose.position.x = setpt[0]
        state_monitor.goal_pose.pose.position.y = setpt[1]
        state_monitor.goal_pose.pose.position.z = setpt[2]

        while not self.reached and not rospy.is_shutdown():
            RATE.sleep()

        rospy.loginfo("Drone reached setpoint!")


if __name__ == "__main__":
    try:
        rospy.init_node("offboard_control", anonymous=True)  # Initialising node
        RATE = rospy.Rate(10.0)  # Setting rate
        rospy.logwarn("Node Started!")

        # Initialising state monitor
        state_monitor = StateMonitor()

        # Defining the setpoints for travel
        setpoints = [
            [0, 0, 10],
            [10, 0, 10],
            [10, 10, 10],
            [0, 10, 10],
            [0, 0, 10],
        ]

        """Initialising publishers and subscribers"""
        #   Position publisher
        position_publisher = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        #   Velocity publisher
        velocity_publisher = rospy.Publisher(
            "mavros/setpoint_position/cmd_vel", Twist, queue_size=10
        )
        rospy.loginfo("Publishers initialised!")

        #   State subscriber
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
        rospy.loginfo("Subscribers initialised!")

        # Initialising drone control
        drone_control = DroneControl()

        """
        NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz,
        so before changing the mode to OFFBOARD, send some dummy setpoints
        """
        for i in range(100):
            position_publisher.publish(state_monitor.goal_pose)
            RATE.sleep()

        # Arming the drone
        while not state_monitor.current_state.armed:
            drone_control.arm_set_mode()
            RATE.sleep()
        rospy.logwarn("Drone Armed!")

        # Switching the state to auto mode
        while not state_monitor.current_state.mode == "OFFBOARD":
            drone_control.drone_set_mode("OFFBOARD")
            RATE.sleep()
        rospy.logwarn("OFFBOARD Mode Activated!")

        # Send setpoints for travel
        for i in setpoints[:]:
            rospy.loginfo(f"New setpoint: {i}")
            drone_control.set_goal(i)

        # Land the drone once done
        while not state_monitor.current_state.mode == "AUTO.LAND":
            drone_control.drone_set_mode("AUTO.LAND")
            RATE.sleep()

        # Waiting for drone to land and disarm
        # while state_monitor.current_state.armed:
        #     pass
        # rospy.logwarn("Drone Landed!")

        # # End the node
        # rospy.logwarn("Node Ended!")
        # drone_control.data_stream.join()
        # sys.exit()

    except rospy.ROSInterruptException:
        pass
