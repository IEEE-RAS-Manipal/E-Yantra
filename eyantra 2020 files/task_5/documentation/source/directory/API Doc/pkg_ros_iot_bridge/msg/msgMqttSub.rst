==============
msgMqttSub.msg
==============

Message file for internal communication and data transfer for the bridge.

The message has three fields:

* :code:`timestamp`: to record the time of transfer
* :code:`topic`: the MQTT topic
* :code:`message`: the actual message/data

.. code-block:: py

   time timestamp
   string topic
   string message

