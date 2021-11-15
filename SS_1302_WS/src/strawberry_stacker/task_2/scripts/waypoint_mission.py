#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name waypoint_Mission which sends the waypoints to drone
in mission mode.
This node publishes and subsribes the following topics:

    Services to be called         Subscriptions		
	/mavros/cmd/arming             /mavros/state
    /mavros/set_mode
    /mavros/mission/push
'''

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class Modes:
    ''' This class is used to call rosservices '''

    def __init__(self):
        pass

# Calling the rosservices

    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
    def set_arm(self):
        # Waiting untill the service starts
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            # Creating a proxy service for rosservice named /mavros/cmd/arming for arming the drone
            arm_service = rospy.ServiceProxy(
                'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            arm_service(True)
        except rospy.ServiceException as er:
            print("Service arming call failed: %s" % er)

    def auto_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to AUTO.MISSION
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flight_mode_service = rospy.ServiceProxy(
                '/mavros/set_mode', mavros_msgs.srv.SetMode)
            flight_mode_service(custom_mode='AUTO.MISSION')
        except rospy.ServiceException as e:
            # and print fail message on failure
            print("service set_mode call failed: %s. AUTO.MISSION Mode could not be set. Check that GPS is enabled %s" % e)

    def wp_push(self, index, wps):
        # Call /mavros/mission/push to push the waypoints
        rospy.wait_for_service('mavros/mission/push')
        try:
            wp_push_service = rospy.ServiceProxy(
                'mavros/mission/push', WaypointPush, persistent=True)
            # start_index = the index at which we want the mission to start
            wp_push_service(start_index=0, waypoints=wps)
            print("Waypoint Pushed")
        except rospy.ServiceException as e:
            # and print fail message on failure
            print("Service takeoff call failed: %s" % e)

    def wp_pull(self, wps):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wp_pull_service = rospy.ServiceProxy(
                'mavros/mission/pull', WaypointPull, persistent=True)
            print(wp_pull_service().wp_received)
            print("Waypoint Pulled")
        except rospy.ServiceException as e:
            print("Service Puling call failed: %s" % e)


class state_moniter:
    '''This monitors the state'''
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

class wp_mission_cnt:
    '''defines functions to set way points'''
    def __init__(self):
        self.w_p = Waypoint()

    def set_way_points(self, frame, command, is_current, autocontinue, param1, param2, param3, param4, x_lat, y_long, z_alt):
        #FRAME_GLOBAL_REL_ALT = 3
        # for more visit http://docs.ros.og/api/mavros_msgs/html/msg/Waypoint.html
        self.w_p.frame = frame
        '''
        VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22
        for other parameters
        go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg
        '''
        self.w_p.command = command
        self.w_p.is_current = is_current
        # enable taking and following upcoming waypoints automatically
        self.w_p.autocontinue = autocontinue
        '''
        To know more about these params,
        visit https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
        '''
        self.w_p.param1 = param1
        self.w_p.param2 = param2
        self.w_p.param3 = param3
        self.w_p.param4 = param4
        self.w_p.x_lat = x_lat
        self.w_p.y_long = y_long
        self.w_p.z_alt = z_alt  # relative altitude.

        return self.w_p


def main():
    ''' Initializing the node and setting wayPoints '''
    rospy.init_node('waypoint_Mission', anonymous=True)  # Initialise rosnode
    rate = rospy.Rate(20.0)

    state_mt = state_moniter()
    m_d = Modes()

    rospy.Subscriber("/mavros/state", State, state_mt.stateCb)

    wayp0 = wp_mission_cnt()
    wayp1 = wp_mission_cnt()
    wayp2 = wp_mission_cnt()
    wayp3 = wp_mission_cnt()

    wps = []  # List to store waypoints

    w = wayp0.set_way_points(3, 22, True, True, 0.0, 0.0,
                           0.0, float('nan'), 19.134641, 72.911706, 10)
    wps.append(w)

    w = wayp1.set_way_points(3, 16, False, True, 0.0, 0.0,
                           0.0, float('nan'), 19.134617, 72.911886, 10)
    wps.append(w)

    w = wayp2.set_way_points(3, 16, False, True, 0.0, 0.0,
                           0.0, float('nan'), 19.134434, 72.911817, 10)
    wps.append(w)

    w = wayp3.set_way_points(3, 21, False, True, 0.0, 0.0,
                           0.0, float('nan'), 19.134423, 72.911763, 10)
    wps.append(w)

    print(wps)
    m_d.wp_push(0, wps)

    m_d.wp_pull(0)
    rospy.Subscriber("/mavros/state", State, state_mt.stateCb)

    # Arming the drone
    while not state_mt.state.armed:
        m_d.set_arm()
        rate.sleep()
        print("ARM!!")

    # Switching the state to auto mode
    while not state_mt.state.mode == "AUTO.MISSION":
        m_d.auto_set_mode()
        rate.sleep()
        print("AUTO.MISSION")


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
