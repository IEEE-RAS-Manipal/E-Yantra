=====================
task5_solution.launch
=====================

This ROS launch file contains the essential code required to start up the entire implementation thus:

Starting up the Gazebo environment:

.. code-block:: xml

   <!-- Launch Task-4 Simulation Environment in Gazebo -->
   <include file="$(find pkg_vb_sim)/launch/task5_simulation.launch" />

Setting up the MoveIt! and RViz environments:

.. code-block:: xml
   
   <!-- Launch MoveIt move_group node for both arms from given pkgs -->
   <include file="$(find pkg_vb_sim)/launch/two_ur5_move_group.launch" />

   <!-- Add scene files to Moveit! Planning Scenes in RViz -->
   <!-- Scene for UR5 Shelf Picker -->
   <group ns="ur5_1">
      <arg name="scene_file" default="$(find pkg_task5)/config/rviz/UR5_1_Shelf.scene"/>
      <node name = "moveit_publish_scene_from_text" pkg= "moveit_ros_planning" type="moveit_publish_scene_from_text" args= "$(arg scene_file)"/>
   </group>
   <!-- Scene for UR5 Bin Sorter -->
   <group ns="ur5_2">
      <arg name="scene_file" default="$(find pkg_task5)/config/rviz/UR5_2_Bins.scene"/>
      <node name = "moveit_publish_scene_from_text" pkg= "moveit_ros_planning" type="moveit_publish_scene_from_text" args= "$(arg scene_file)"/>
   </group>

Notice that the :code:`group` notation is a namespace used to group the assets for each UR5, extracting the appropriate data from the provided packages.

Starting up the ROS nodes:

.. code-block:: xml

   <!-- Executing ROS nodes for overall control -->

   <!-- Loading ROS-IoT parameters into the parameter server -->
   <rosparam file="$(find pkg_ros_iot_bridge)/config/config_pyiot.yaml" command="load"/>

   <!-- Start the ROS-IoT Bridge node -->
   <node name="node_action_server_ros_iot_bridge" pkg="pkg_ros_iot_bridge" type="node_action_server_ros_iot_bridge.py" output="screen"/>

   <!-- Node to obtain package colours -->
   <node name="node_t5_package_detect" pkg="pkg_task5" type="node_package_detect.py" output="screen" />
   <!-- Node to control the Shelf UR5 -->
   <node name= "node_t5_ur5_1" pkg= "pkg_task5" type="node_t5_ur5_1.py" output="screen"/>
   <!-- Node to control the Bins UR5 -->
   <node name= "node_t5_ur5_2" pkg= "pkg_task5" type="node_t5_ur5_2.py" output="screen"/>

