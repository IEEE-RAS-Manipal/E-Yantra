#!/usr/bin/env python3

"""
Task 2.2 module
"""

import threading
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
import mavros_msgs.srv


class StateMonitor:
    """
    StateMonitor State monitor with callback functions

    This class contains callback functions of the subscriber of this node that track the state and
    pose of the drone during operation.
    """

    def __init__(self) -> None:
        """
        __init__ Initialises class attributes

        This method initialises the class attributes of the StateMonitor class, namely the state
        of the drone, and the drone's current pose during operation.
        """
        self.current_state = State()  #   State object for tracking drone state
        self.goal_pose = (
            PoseStamped()
        )  #   PoseStamped object to track current drone pose
        self.event_log = threading.Event()

        """Initialising publishers"""
        #   Position publisher
        self.position_publisher = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        #   Velocity publisher
        self.velocity_publisher = rospy.Publisher(
            "mavros/setpoint_position/cmd_vel", Twist, queue_size=10
        )

        """Initialising subscriber"""
        #   State subscriber
        self.state_subscriber = rospy.Subscriber(
            "mavros/state", State, self.state_callback, queue_size=10
        )
        #   Pose subscriber
        self.pose_subscriber = rospy.Subscriber(
            "mavros/local_position/pose", self.pose_callback, queue_size=10
        )

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

        :param pose: The current pose of the drone.
        :type pose: PoseStamped
        """

        def is_near(pose, current, goal):
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
            drone_control.done = True
            self.event_log.set()


class DroneControl:
    """
    This class sends position targets to FCU's position controller
    """

    def __init__(self) -> None:
        try:
            data_stream = threading.Thread(target=drone_control.drone_data_stream)
            data_stream.start()
        except Exception:
            rospy.logwarn("Error: Unable to start thread")
        self.done = False
        self.goal_pose = PoseStamped()

    def arm_set_mode(self) -> None:
        """
        arm_set_mode Arms the drone

        Arms the drone for control
        """
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service("mavros/cmd/arming")  # Waiting untill the service starts

        try:
            arm_service = rospy.ServiceProxy(
                "mavros/cmd/arming", mavros_msgs.srv.CommandBool
            )
            # Creating a proxy service for the rosservice named /mavros/cmd/arming to arm the drone
            arm_service(True)
        except rospy.ServiceException as exception:
            print("Service arming call failed: %s" % exception)

    def offboard_set_mode(self):
        """
        offboard_set_mode Initialise offboard mode

        Initialises offboard mode for the drone setpoint control
        """
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # Waiting untill the service starts
        rospy.wait_for_service("mavros/set_mode")
        try:
            set_mode = rospy.ServiceProxy("mavros/set_mode", mavros_msgs.srv.SetMode)
            set_mode(custom_mode="OFFBOARD")
        # and print fail message on failure
        except rospy.ServiceException as exception:
            # and print fail message on failure
            print(
                "service set_mode call failed: %s. Check that GPS is enabled %s"
                % exception
            )

    def drone_data_stream(self):
        """
        drone_data_stream Establishes a data stream

        Continuous data stream to the drone for setpoint transmission
        """

        while not rospy.is_shutdown():
            state_monitor.position_publisher.publish(self.goal_pose)
            RATE.sleep()

    def set_goal(self, setpt: list, wait=True):
        """
        set [summary]

        [extended_summary]

        :param x: [description]
        :type x: [type]
        :param y: [description]
        :type y: [type]
        :param z: [description]
        :type z: [type]
        :param delay: [description], defaults to 0
        :type delay: int, optional
        :param wait: [description], defaults to True
        :type wait: bool, optional
        """
        self.done = False
        self.goal_pose.pose.position.x = setpt[0]
        self.goal_pose.pose.position.y = setpt[1]
        self.goal_pose.pose.position.z = setpt[2]

        if wait:
            while not self.done and not rospy.is_shutdown():
                RATE.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("offboard_control", anonymous=True)  # Initialising node
        state_monitor = (
            StateMonitor()
        )  # Initialising state monitor, publishers and subscribers
        drone_control = DroneControl()  # Initialising drone control
        RATE = rospy.Rate(10.0)  # Setting rate

        # Defining the setpoints for travel
        setpoints = [[0, 0, 0], [1, 1, 1]]

        """
        NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
        """
        for i in range(100):
            state_monitor.position_publisher.publish(state_monitor.goal_pose)
            RATE.sleep()

        # Arming the drone
        while not state_monitor.current_state.armed:
            drone_control.arm_set_mode()
            RATE.sleep()
        rospy.logwarn("Armed!!")

        # Switching the state to auto mode
        while not state_monitor.current_state.mode == "OFFBOARD":
            drone_control.offboard_set_mode()
            RATE.sleep()
        rospy.logwarn("OFFBOARD mode activated")

        for i in setpoints[:]:
            drone_control.set_goal(i)

    except rospy.ROSInterruptException:
        pass
