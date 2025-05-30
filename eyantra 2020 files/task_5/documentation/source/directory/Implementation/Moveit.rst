================
MoveIt! and RViz
================

The MoveIt! + RViz platform for robotics visualisation and manipulation is the primary method for implementing many robotics applications. As a bundled package, it has robust integration with each other and easy to implement.

----

MoveIt!
*******

.. figure:: Images/MoveIt.png
   :align: center

   The MoveIt! Setup Assistant that generates controllers for the robot off its URDF.

The MoveIt! Motion Planning framework is a robotics manipulation platform that enables robots to perform operations on objects, conduct autonomous motion planning, etc. that can also be seen in simulation environment like Gazebo. The crux of MoveIt! is in the controllers generated by the pipeline with the help of the `MoveIt! Setup Assistant`_, on providing a URDF file of the robot and specifying joints and preset poses.

.. _`MoveIt! setup assistant`: http://docs.ros.org/en/hydro/api/moveit_setup_assistant/html/doc/tutorial.html

The MoveIt! robot controllers enable the robot to move in Gazebo according to the motion plan set in RViz. The default controllers provided were used for the implementation. The ``FollowJointTrajectory`` controller, which implements the joint trajectory method of joint interpolation for robotic joints, is the default controller used due to its easy implementation and fast compute times.

The config.yml file for the controller was written thus:

.. code-block:: xml

   controller_manager_ns: /
   controller_list:
   - name: ur5_controller
     action_ns: follow_joint_trajectory         
     type: FollowJointTrajectory                --> Selecting the type of motion controller
     joints:                                    --> Designate the joints affected by this control scheme
     - ur5_shoulder_pan_joint
     - ur5_shoulder_lift_joint
     - ur5_elbow_joint
     - ur5_wrist_1_joint
     - ur5_wrist_2_joint
     - ur5_wrist_3_joint

----

RViz
****

.. figure:: Images/RViz.png
   :align: center

   The RViz Planning Scenes for both UR5s.

RViz is the primary robot visualisation environment bundled with the MoveIt! pipeline that enables us to view the world as seen through the robot's eyes, providing a greater insight into how it can react to stimuli and perform tasks.

.. _scene-file:

Scene Files
-----------

The environments in RViz seen in the figure were built to replicate the Gazebo environment, by adding objects one-by-one and then `exported to a scene file`_ that can be read from the next time RViz launches to bring the objects in the environment automatically. For example, the boxes on the shelf in the UR5_1's environment are written in the scene file thus:

.. code-block:: c

   * packagen00         --> The name of the object
   1                    --> The number of objects
   box                  --> The type of primitive shape
   0.16 0.16 0.16       --> The dimensions
   0.28 -0.42 1.91      --> The postiion in metres along x,y,z
   0 0 0 1              --> The orientation of the object
   0 0 0 0

.. _`exported to a scene file`: https://groups.google.com/g/moveit-users/c/_3h3B_fNNao

.. _TFframe:

TF Frames
---------

.. figure:: Images/TFframe.png
   :align: center

   The TF frame of a package in RViz, as seen by the logical camera in Gazebo.

`TF <http://wiki.ros.org/tf>`_ is a package that enables us to keep track of coordinate frames of objects when detected through the logical camera in Gazebo. TF was used with the UR5_2 through the :meth:`camera_callback() <node_t5_ur5_2.UR5Two.camera_callback>` method in :mod:`node_t5_ur5_2` to detect the packages arriving on the conveyor to pinpoint the location so the arm can travel to the package to pick it up.
