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
            [-1, 1, 4],
            [2, 17, 4],
            [15, 17, 4],
            [30, 17, 4],
            [45, 17, 4],
            [60, 17, 4],
            [-1, 1, 3],
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
                [14.7, -4.94, 2])  # Placing package
        rospy.loginfo("Back to scanning...")
        drone1.drone_control.drone_set_goal(
            [15.55, 1, 3], True)  # turning point
        drone1.drone_control.drone_set_goal([1, 24, 3], True)
        drone1.drone_control.drone_row_patrol(7)
        '''
        drone1.drone_control.drone_set_goal([20, 24, 3])
        '''
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
            [-1, 61, 4],
            [1, 49, 4],
            [15, 49, 4],
            [30, 49, 4],
            [45, 49, 4],
            [60, 49, 4],
            [0, 0, 4],
        ]

        drone2.drone_control.drone_set_goal(setpoints[0], override=True)
        drone1.drone_control.drone_row_patrol(8)
        '''
        drone2.drone_control.drone_set_goal(setpoints[1], override=True)
        drone2.drone_control.drone_set_goal(setpoints[2])
        '''

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
        drone1.drone_control.drone_row_patrol(13)
        '''
        drone2.drone_control.drone_set_goal([20, 29, 4])
        '''
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
