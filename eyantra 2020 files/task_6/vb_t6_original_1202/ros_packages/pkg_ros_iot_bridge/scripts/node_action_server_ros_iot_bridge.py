#!/usr/bin/env python
"""
ROS Node - Action Server - IoT ROS Bridge
"""
import json
import threading
import requests
import actionlib
import rospy

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgMqttSub

from pyiot import iot


def on_cancel(goal_handle):
    """
    This function will be called when Goal Cancel request is send to the Action Server
    """
    rospy.loginfo("Received cancel request.")
    goal_handle.get_goal_id()


class RosIotBridgeActionServer(object):
    """
    Bridge between ROS and the Google Sheet using MQTT Protocol
    """
    param_config_iot = rospy.get_param('config_iot')

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the function pointer which
                             points to a function which will be called
                             when the Action Server receives a Goal.

            * self.on_cancel - It is the function pointer which points
                               to a function which will be called
                               when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server

        self._config_mqtt_server_url = self.param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = self.param_config_iot['mqtt']['server_port']
        self._config_mqtt_qos = self.param_config_iot['mqtt']['qos']
        self._config_google_apps_spread_sheet_id = \
            self.param_config_iot['google_apps']['spread_sheet_id']
        self._config_google_apps_submission_spread_sheet_id = \
            self.param_config_iot['google_apps']['submission_spread_sheet_id']
        print self.param_config_iot

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be
        # published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub)
        # to get messages from MQTT Subscription.

        self._handle_ros_pub = rospy.Publisher(
            self.param_config_iot['mqtt']['sub_cb_ros_topic'], msgMqttSub, queue_size=10)

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros)
        # which is defined in 'config_ros_iot.yaml'.
        # self.mqtt_sub_callback() function will be called
        # when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self.param_config_iot['mqtt']['topic_sub'],
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        """
        Callback function for MQTT request subscriber
        """
        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)
        print client
        print userdata

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    def on_goal(self, goal_handle):
        """
        This function will be called when Action Server receives a Goal
        """
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if goal.protocol == "mqtt":

            if (goal.mode == "pub") or (goal.mode == "sub"):
                goal_handle.set_accepted()

                # Start a new thread to process new goal from the client
                # (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which
                # points to a function that will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return

    def process_goal(self, goal_handle):
        """
        This function is called is a separate thread to process Goal.
        """
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                    self.spreadsheet_push(goal_handle)
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if result.flag_success:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def spreadsheet_push(self, goal_handle):
        """
        Pushes to the Google Sheet
        """

        goal = goal_handle.get_goal()

        # PERSONAL SPREADSHEET

        goal_string = json.loads(str(goal.message))

        personal_parameters = goal_string

        personal_url = "https://script.google.com/macros/s/" + \
                       self._config_google_apps_spread_sheet_id + "/exec"

        personal_response = requests.get(personal_url, params=personal_parameters)
        print("Personal Spreadsheet status:", personal_response.content)

        # EYRC SPREADSHEET

        eyrc_parameters = goal_string

        eyrc_url = "https://script.google.com/macros/s/" + \
              self._config_google_apps_submission_spread_sheet_id + "/exec"

        eyrc_response = requests.get(eyrc_url, params=eyrc_parameters)
        print eyrc_response.content
        print("e-Yantra Spreadsheet status:", eyrc_response.content)


if __name__ == '__main__':
    rospy.init_node('node_action_server_ros_iot_bridge')

    RosIotBridgeActionServer()

    rospy.spin()
