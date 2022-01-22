#!/usr/bin/env python3
'''This script makes the turtlesim turtle traverse a circle.'''

# Importing essential packages
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Defining constants
THETA = 0


def circle(control, path_end):
    '''
    This function sets the velocities for the circle.

    Parameters:
        control (Twist): The Twist data object that will manipulate the turtle's velocities.
        path_end (Boolean): Indicates the turtle has reached the end of it's path.

    Returns:
        Twist: The modified data object with the velocities to be published to the turtle.
    '''
    # Preparing the values to set
    radius = 2
    linear_x_velocity = 2

    # Setting the values for traversal
    control.linear.x = linear_x_velocity
    control.angular.z = linear_x_velocity/radius

    # Setting the values to stop traversal
    if path_end:
        control.linear.x = 0
        control.angular.z = 0

    return control


def controller():
    '''This function starts and stops the turtle's motion.'''
    # Configuring the Subscriber
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # Defining the start/stop flags
    path_end = False
    path_start = False

    # Obtaining the velocities to be entered
    twist = Twist()
    twist = circle(twist, path_end)

    # Control loop
    start_time = rospy.get_time()
    while twist.linear.x != 0:

        # Flag the start of the traversal
        if THETA > 1 and not path_start:
            path_start = True
        # Flag the end of the traversal
        elif abs(THETA) <= 0.05 and path_start:
            path_end = True
            twist = circle(twist, path_end)

        # Publish and log data
        TURTLE_CONTROL.publish(twist)
        rospy.loginfo('Moving in a circle\n%s', rospy.get_time()-start_time)
        RATE.sleep()

    # Log the end of traversal
    rospy.loginfo('Goal Reached\n%s', rospy.get_time()-start_time)


def pose_callback(pose_data):
    '''
    This callback function is called everytime the Pose data of the turtle is updated.

    Parameters:
        pose_data (Pose): The update Pose data of the turtle.

    Constants:
        THETA (float): The angle orientation of the turtle.
    '''
    global THETA
    THETA = round(pose_data.theta, 3)


if __name__ == '__main__':
    try:
        rospy.init_node('Controller', anonymous=True)

        # Configuring the Publisher
        TURTLE_CONTROL = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        RATE = rospy.Rate(10)  # Setting Publishing rate

        controller()

    except rospy.ROSInterruptException():
        pass
