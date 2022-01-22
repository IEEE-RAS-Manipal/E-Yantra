=================
config_pyiot.yaml
=================

Contains preloaded data about server URLs and credentials to access during ROS-IoT bridge operation.

The MQTT protocol requires a server URL, connection port, and topics to subscribe to and publish data from, for a specific client.
The Google sheet webapp ID allows us to access the sheet to push data to it programmatically..

.. code-block:: py

   # config_iot_ros.yaml 
   # IoT Configuration
   config_iot:
     mqtt:
       server_url: "broker.mqttdashboard.com"        # http://www.hivemq.com/demos/websocket-client/
       # server_url: "test.mosquitto.org"        # Alternative to HiveMQ
       server_port: 1883
       topic_sub: "/eyrc/vb/isAmiTvb/orders"           # <unique_id> = isAmiTvb
       topic_pub: "/eyrc/isAmiTvb/ros_to_iot"          # <unique_id> = isAmiTvb
       qos: 0

       sub_cb_ros_topic: "/ros_iot_bridge/mqtt/sub"   # ROS nodes can listen to this topic to receive data from MQTT

     google_apps:
       spread_sheet_id: AKfycbw7FjbyUUs_7_E1Ky3rKNAcBDEX59OXJdg3s3Tf1BgdVGJtUoN0MuD2UA
       submission_spread_sheet_id: AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7

