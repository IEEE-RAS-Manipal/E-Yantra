===========
Theme Arena
===========

.. figure:: Arena.png
   :align: center

   The warehouse for the theme.

The arena for the theme implementation is a remote warehouse modeled in Gazebo, as shown in the figure. The following elements have been provided for the task:

UR5 Robotic Arms
****************
The warehouse contains two `UR5 Robotic Arms from Universal Robots`_, which are lightweight industrial robots tackling medium level industry operations with maximum flexibility:

* UR5_1: The first industrial robotic arm is used to pick up objects from the shelf of the warehouse and place it on a conveyor belt.

* UR5_2: The second industrial robotic arm is placed at the end of the conveyor belt, where there are also three coloured bins matching the colour of the package. The arm picks up the objects from the moving conveyor belt and segregates them in respective bins.

.. _`UR5 Robotic Arms from Universal Robots`: https://www.universal-robots.com/products/ur5-robot/

Logical Camera
**************
The two blue cameras placed above the conveyor belt help the UR5s :ref:`identify the packages <LogicalCamera>` and segregate them accordingly.

Camera
******

.. image:: Camera.png
   :align: center

This camera is a 2D camera facing the packages on the shelf. The role of this camera is to use :doc:`Computer Vision techniques <../API Doc/pkg_task5/scripts/node_package_detect>` to identify packages on the shelf.


Shelf and Conveyor Belt
***********************
The Shelf and Conveyor Belt are an abstraction of an inventory shelf and conveyor belt in a warehouse for this implementation.

Sorting Bins and Packages
*************************
The three sorting bins located around the UR5_2 are coloured according to the priority of the packages being put in it:

* Red: High Priority packages like Medicines
* Yellow: Medium Priority packages like Food
* Green: Low Priority packages like clothes
