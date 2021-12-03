# usr/bin/python3

"""
"""

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros import setpoint as SP
import mavros_msgs.srv
import thread
import threading


class StateMonitor:
    """
    StateMonitor State monitor with callback functions

    This class contains callback functions of the subscriber of this node that track the state and pose of the drone during operation.
    """

    def __init__(self) -> None:
        """
        __init__ Initialises class attributes

        This method initialises the class attributes of the StateMonitor class, namely the state of the drone, and the drone's current pose during operation.
        """
        self.current_state = State()  #   State object for tracking drone state
        self.goal_pose = (
            PoseStamped()
        )  #   PoseStamped object to track current drone pose
        self.done = False
        self.event_log = threading.Event()

    def state_callback(self, state: State) -> None:
        """
        state_callback Callback function for state_subscriber

        This is the callback function for the state_subscriber Subscriber for the /mavros/state topic.

        :param state: The current state/mode of the drone.
        :type state: State
        """
        self.current_state = state  #   Recording current state of the drone

    def pose_callback(self, curr_pose: PoseStamped) -> None:
        """
        pose_callback Callback function for pose_subscriber

        This is the callback function for the pose_subscriber Subscriber for the /mavros/local_position/pose topic.

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
            self.done = True
            self.done_evt.set()


def setArm() -> None:
    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
    rospy.wait_for_service("mavros/cmd/arming")  # Waiting untill the service starts

    try:
        armService = rospy.ServiceProxy(
            "mavros/cmd/arming", mavros_msgs.srv.CommandBool
        )  # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
        armService(True)
    except rospy.ServiceException as e:
        print("Service arming call failed: %s" % e)


def offboard_set_mode():
    # Call /mavros/set_mode to set the mode the drone to OFFBOARD
    # Waiting untill the service starts
    rospy.wait_for_service("mavros/set_mode")
    try:
        set_mode = rospy.ServiceProxy("mavros/set_mode", mavros_msgs.srv.SetMode)
        set_mode(custom_mode="OFFBOARD")
    # and print fail message on failure
    except rospy.ServiceException as e:
        # and print fail message on failure
        print(
            "service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled %s"
            % e
        )


class SetpointPosition:
    """
    This class sends position targets to FCU's position controller
    """

    def setpoint_data_stream(self):
        rate = rospy.Rate(10)  # 10hz

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now(),
            ),  # stamp should update
        )

        while not rospy.is_shutdown():
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z

            Publishers.position_publisher.publish(msg)
            RATE.sleep()

    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z

        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()

        time.sleep(delay)


class Publishers:
    """
    Publishers Initialises Publisher Nodes

    This class initialises and tracks the publisher nodes used in this module.
    """

    def __init__(self) -> None:
        """
        __init__ Initialises class attributes

        This method initialises the publishers.
        """
        #   Position publisher
        self.position_publisher = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        #   Velocity publisher
        self.velocity_publisher = rospy.Publisher(
            "mavros/setpoint_position/cmd_vel", Twist, queue_size=10
        )


class Subscribers:
    """
    Subscribers Initialises Subscriber Nodes

    This class initialises and tracks the subscribers used in this module.
    """

    #   State subscriber
    state_subscriber = rospy.Subscriber(
        "mavros/state", State, StateMonitor.state_callback, queue_size=10
    )
    #   Pose subscriber
    pose_subcriber = rospy.Subscriber(
        "mavros/local_position/pose", StateMonitor.pose_callback, queue_size=10
    )


if __name__ == "main":
    try:
        rospy.init_node("offboard_control", anonymous=True)  #   Initialising node
        state_monitor = StateMonitor()  # Initialising state monitor
        setpoint_control = SetpointPosition()

        publishers = Publishers()  #   Initialise publishers
        subscribers = Subscribers()  #   Initialise subscribers
        RATE = rospy.Rate(10.0)  #   Setting rate

        """
        NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
        """
        for i in range(100):
            publishers.position_publisher.publish(state_monitor.goal_pose)
            RATE.sleep()

        # Arming the drone
        while not state_monitor.current_state.armed:
            setArm()
            RATE.sleep()
        rospy.logwarn("Armed!!")

        # Switching the state to auto mode
        while not state_monitor.current_state.mode == "OFFBOARD":
            offboard_set_mode()
            RATE.sleep()
        rospy.logwarn("OFFBOARD mode activated")

        # Start a new thread for navigation
        try:
            thread.start_new_thread(setpoint_control.navigate)
        except:
            rospy.logwarn("Error: Unable to start thread")

        self.done = False
        self.done_evt = threading.Event()

    except rospy.ROSInterruptException:
        pass
