=========
pkg_task5
=========

This package contains the main execution files for the implementation, controlling the UR5s and processing orders received from the client.

config
******
These folders contain preloaded data and files for the other files in this package to access:

* :doc:`config/rviz`: Scene object files
* :doc:`config/saved_trajectories`: Saved trajectories for the UR5s

.. toctree::
   :hidden:

   config/rviz
   config/saved_trajectories

launch
******
This launch file starts up the whole implementation:

* :doc:`launch/task5_solution`: Main execution file

.. toctree::
   :hidden:

   launch/task5_solution

scripts
*******
These nodes control the overall implementation:

* :doc:`scripts/lib_task5`: Library for UR5s
* :doc:`scripts/node_t5_ur5_1`: Controls the UR5_1
* :doc:`scripts/node_t5_ur5_2`: Controls the UR 5_2
* :doc:`scripts/node_package_detect`: Processes the colours of packages
* :doc:`scripts/node_iot_action_client`: Receives orders from the server

.. toctree::
   :hidden:

   scripts/lib_task5
   scripts/node_t5_ur5_1
   scripts/node_t5_ur5_2
   scripts/node_package_detect
   scripts/node_iot_action_client
