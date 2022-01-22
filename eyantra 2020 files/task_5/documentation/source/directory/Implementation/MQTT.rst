=============
MQTT Protocol
=============

The MQTT Protocol is an extremely simple and lightweight messaging protocol, designed for constrained devices and low-bandwidth, high-latency or unreliable networks, such as the remote warehouse in this implementation. The publisher-subscriber model also allows MQTT communication in the one-one, one-many and many-one setups.

The essential elements of this pipeline are:

MQTT Publisher
**************
The publisher acts as a server who is used to 'publish' messages or here, topics to the client side through a broker.

MQTT Topics
***********
This is a UTF-8 string that the broker uses to filter messages for each connected client. The topic consists of one or more levels. Each 'topic level' is separated by a forward slash which is known as a topic separator.

MQTT Broker
***********
The MQTT broker is primarily responsible for receiving all messages, filtering the messages, decide who is interested in them and then publishing the message to all subscribed clients.

MQTT Subscriber
***************
The MQTT subscriber 'subscribes' to topic published on to the broker by the publisher. It acts as a client for receiving and generating a record of the messages published.
