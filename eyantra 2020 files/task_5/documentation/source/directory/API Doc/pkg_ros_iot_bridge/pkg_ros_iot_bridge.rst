==================
pkg_ros_iot_bridge
==================

This package contains the framework required for integration with MQTT protocol to the client server and GSuite.

config
******
Contains backend data about MQTT and Google Sheet push targets:

* :doc:`config/config_pyiot`

.. toctree::
   :hidden:

   config/config_pyiot

msg
***
``msg`` format that signals are sent in:

* :doc:`msg/msgMqttSub`

.. toctree::
   :hidden:

   msg/msgMqttSub

scripts
*******
Controls the backend integration with MQTT:

* :doc:`scripts/node_action_server_ros_iot_bridge`

.. toctree::
   :hidden:

   scripts/node_action_server_ros_iot_bridge
