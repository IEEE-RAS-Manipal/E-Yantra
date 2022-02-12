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
            package_pos[0] = package_pos[0] + 1
            self.drone_set_goal(package_pos, True, True, 0.3)

            self.stream_switch = True  # Switch to velocity setpoint transmission

            while self.drone_monitor.current_state.armed and self.drone_monitor.current_pose.pose.position.z > 0.3:
                # Real-time position of ArUco marker
                [aruco_cx, aruco_cy] = self.drone_monitor.aruco_centre[0][:]

                # Tweaking velocity of the drone using the exp(0.4x-3) function
                vel[0] = exp(
                    0.4
                    * abs(
                        package_pos[0] -
                        self.drone_monitor.current_pose.pose.position.x
                    )
                    - 3
                )
                vel[1] = exp(
                    0.4
                    * abs(
                        package_pos[1] -
                        self.drone_monitor.current_pose.pose.position.y
                    )
                    - 3
                )
                # Velocity along the Z axis follows exp(0.4z-1) curve
                vel[2] = exp(
                    (0.4
                     * self.drone_monitor.current_pose.pose.position.z) - 1)

                # If ArUco is in this range, begin descending
                quad_x = 200
                quad_y = 200

                if (aruco_cx in range(170, 230)) and (aruco_cy in range(170, 230)) and self.drone_monitor.current_pose.pose.position.z > 1.5:
                    self.drone_monitor.goal_vel.linear.z = -vel[2]

                if self.drone_monitor.current_pose.pose.position.z < 2:
                    quad_x = 200
                    quad_y = 270
                    self.drone_monitor.goal_vel.linear.z = -exp(
                        (0.4
                         * self.drone_monitor.current_pose.pose.position.z) - 3)

                    if ((aruco_cx in range(192, 208)) and (aruco_cy in range(262, 278)) and self.drone_monitor.current_pose.pose.position.z > 1) or self.drone_monitor.current_pose.pose.position.z <= 0.5:
                        self.drone_monitor.goal_vel.linear.x = 0
                        self.drone_monitor.goal_vel.linear.y = 0
                        while self.drone_monitor.current_pose.pose.position.z > 0.3:
                            self.drone_monitor.goal_vel.linear.z = -exp(
                                (0.4
                                 * self.drone_monitor.current_pose.pose.position.z) - 2)
                        while not self.drone_monitor.gripper_state:
                            print(self.drone_monitor.gripper_state)
                            RATE.sleep()
                        rospy.loginfo("Attempting to grip...")
                        # Activating the gripper
                        flag = self.drone_gripper_attach(True)
                        if flag == 1:
                            rospy.loginfo(
                                f"\033[92mDrone #{self.drone_id+1} package picked! Proceeding to dropoff point!\033[0m")
                        else :
                            package_pos[2]=1.5
                            self.drone_set_goal(package_pos, True, True, 0.2)

                            continue
                        # self.drone_shutdown()

                if self.drone_monitor.current_pose.pose.position.z in range(1, 3):
                    self.drone_monitor.goal_vel.linear.z = 0
                    # Checking the coordinates again and locking the position
                    if (aruco_cx in range(195, 205)) and (aruco_cy in range(195, 205)):
                        package_pos[0] = self.drone_monitor.current_pose.pose.position.x
                        # adding offset
                        package_pos[1] = self.drone_monitor.current_pose.pose.position.y
                        self.drone_monitor.goal_vel.linear.x = 0
                        self.drone_monitor.goal_vel.linear.y = 0
                        print(" Insideeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
                        package_pos[2] = 1.7

                        self.stream_switch = False  # beginning to transmit setpoints
                        self.drone_set_goal(package_pos, True, True, 0.1)
                        self.stream_switch = True

                if aruco_cx > quad_x:
                    self.drone_monitor.goal_vel.linear.x = vel[0]
                elif aruco_cx < quad_x:
                    self.drone_monitor.goal_vel.linear.x = -vel[0]

                if aruco_cy > quad_y:
                    self.drone_monitor.goal_vel.linear.y = -vel[1]
                elif aruco_cy < quad_y:
                    self.drone_monitor.goal_vel.linear.y = vel[1]
                RATE.sleep()

            self.stream_switch = False  # Switch back to normal setpoint tranmission

            # Taking off from location
            self.drone_startup()
            package_pos[2] = 3
            self.drone_set_goal(package_pos, True, True)
            if self.drone_id == 0:
                self.drone_set_goal([15.55, 0, 3], True,
                                    False, 1)  # turning point
                self.drone_package_place(truck_inventory(1))
            elif self.drone_id == 1:
                self.drone_set_goal(
                    [57.35, 62, 4], True, False, 1)  # Turning point
                self.drone_package_place(truck_inventory(0))  # Placing package

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
            # self.drone_shutdown()
            place_pos[2] = 1.8
            self.drone_set_goal(place_pos, True, False)
            # Deactivating the gripper
            rospy.loginfo("Deactivating gripper...")
            self.drone_gripper_attach(False)
            rospy.loginfo("Gripper deactivated.")
            rospy.loginfo(
                f"\033[92mD  Drone #{self.drone_id+1}Package placed!\033[0m")
            # Taking off

            place_pos[2] = 3
            self.drone_set_goal(place_pos, True, False)
            self.drone_monitor.aruco_check = False
            self.drone_monitor.aruco_centre[0] = [0, 0]
            print(" End of package place function! ")
            if self.drone_id == 0:
                self.drone_set_goal([15.55, 0, 3], True,
                                    False, 0.5)  # turning point
            elif self.drone_id == 1:
                self.drone_set_goal(
                    [57.35, 62, 4], True, False, 0.5)  # Turning point
            self.done_with_row = True

        def drone_gripper_attach(self, activation: bool) -> None:
            """
            drone_gripper_attach Drone Gripper Control
            Controls the gripper of the drone for picking and placing of objects with the drone.
            :param activation: Command sent to activate or deactivate the gripper
            :type activation: bool
            """
            grip_status = False
            #response = None
            try:
                while not grip_status:
                    if activation:
                        grip_status = self.gripper_service(activation).result
                        RATE.sleep
                    else:
                        self.gripper_service(activation)
                        grip_status = True
                return 1
            except rospy.ServiceException:
                return 0