#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode.
See the documentation for offboard mode in px4 here() to understand more about offboard mode
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                    Subscriptions
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose


'''

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import *
import numpy as np
from six.moves import xrange


class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService = rospy.ServiceProxy(
                'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies

    def set_mode_function(self, mode):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            set_mode_srv = rospy.ServiceProxy(
                'mavros/set_mode', mavros_msgs.srv.SetMode)
            set_mode_srv(custom_mode=mode)
        # and print fail message on failure
        except rospy.ServiceException as e:
            # and print fail message on failure
            print(
                "service set_mode call failed: %s. {0} Mode could not be set. Check that GPS is enabled %s" % e, mode)


class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.from_drone = PoseStamped()
        # Creating connection to service
        self.grip_srv = rospy.ServiceProxy('/activate_gripper', Gripper)
        self.grip_srv.wait_for_service()

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers
    def posPub_cb(self, msg):
        self.from_drone.pose.position.x = msg.pose.position.x
        self.from_drone.pose.position.y = msg.pose.position.y
        self.from_drone.pose.position.z = msg.pose.position.z

    def proximity_checker(self, x, y, z):
        ''' checks if the sent coordinates in the argument are close to the drone's real-time coordinates and returns True if it's close enough'''
        offset = 0.1  # offset is in metres
        desired = np.array((x, y, z))
        actual = np.array((self.from_drone.pose.position.x,
                           self.from_drone.pose.position.y,
                           self.from_drone.pose.position.z))
        if np.linalg.norm(desired - actual) < offset:
            return True
        else:
            return False

    def grip_check(self, msg):
        self.grip_near = msg.data

    def hold_box(self, value):
        req = GripperRequest()
        req.activate_gripper = value
        resp = self.grip_srv(req)
        return resp.result


def main():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher(
        'mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher(
        'mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints = [[0, 0, 3], [3, 0, 3],
                 [3, 0, 3], [3, 3, 3], [3, 3, 3], [0, 0, 3]]  # List to setpoints

    # Similarly initialize other publishers

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 5
    vel.linear.y = 5
    vel.linear.z = 5

    # Similarly add other containers

    # Initialize subscriber
    rospy.Subscriber("/mavros/state", State, stateMt.stateCb)

    # Similarly initialize other subscribers
    pos_pub = rospy.Subscriber(
        "/mavros/local_position/pose", PoseStamped, stateMt.posPub_cb)

    # Subscribing to the gripper
    gripper_check = rospy.Subscriber(
        '/gripper_check', String, stateMt.grip_check)

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state.mode == "OFFBOARD":
        ofb_ctl.set_mode_function('OFFBOARD')
        rate.sleep()
    print("OFFBOARD mode activated")

    # Publish the setpoints
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint
        Step 2: Then wait till the drone reaches the setpoint,
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done


        Write your algorithm here
        '''
        for i in xrange(len(setpoints)):
            # Setting the setpoint
            pos.pose.position.x = setpoints[i][0]
            pos.pose.position.y = setpoints[i][1]
            pos.pose.position.z = setpoints[i][2]

            # Wait till the drone reaches the point
            while not (stateMt.proximity_checker(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z)):
                # keeps publishing till setpoint is reached
                local_pos_pub.publish(pos)
                local_vel_pub.publish(vel)
                print("inside proxim check loop")
                rate.sleep()

            if i == 1:
                print("I'm in the if loop")

                while not stateMt.state.mode == "AUTO.LAND":
                    ofb_ctl.set_mode_function('AUTO.LAND')
                    rate.sleep()
                print("Land mode activated")

                rospy.sleep(8)

                while not stateMt.grip_near:
                    # correcting position
                    local_pos_pub.publish(pos)
                    print(stateMt.grip_near)
                print("The gripper is near")

                response = False
                while not response:
                    response = stateMt.hold_box(True)
                    print("Box not attached yet")

                    # the box is attached
                print("Box attached")

                for i in range(100):
                    local_pos_pub.publish(pos)
                    rate.sleep()

                # Arming the drone
                while not stateMt.state.armed:
                    ofb_ctl.setArm()
                    rate.sleep()
                print("Armed!!")

                # Switching the state to auto mode
                while not stateMt.state.mode == "OFFBOARD":
                    ofb_ctl.set_mode_function('OFFBOARD')
                    rate.sleep()
                print("OFFBOARD mode activated")

            elif i == 3:
                print("inside i = 3 looooOOOOOOoOOoOOOOoOp")
                while not stateMt.state.mode == "AUTO.LAND":
                    ofb_ctl.set_mode_function('AUTO.LAND')
                    rate.sleep()
                print("Land mode activated")
                rospy.sleep(8)

                stateMt.hold_box(False)
                print("Dropped the box")
                for i in range(100):
                    local_pos_pub.publish(pos)
                    rate.sleep()

                # Arming the drone
                while not stateMt.state.armed:
                    ofb_ctl.setArm()
                    rate.sleep()
                print("Armed!!")

                # Switching the state to auto mode
                while not stateMt.state.mode == "OFFBOARD":
                    ofb_ctl.set_mode_function('OFFBOARD')
                    rate.sleep()
                print("OFFBOARD mode activated")

            # publishing the next setpoint
            print("going to the next setopint!!!!!!!!!!!!!!!!")

        # landing once the path is traced
        ofb_ctl.set_mode_function("AUTO.LAND")

        # The drone disarms automatically after landing


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
