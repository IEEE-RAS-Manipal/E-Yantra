========================
GSuite, Sheets and Gmail
========================

The Google ecosystem made integration of the MQTT protocol server-client pipeline and the Dashboard database integration a breeze, with its inbuilt scripts using Google Sheets Script Editor.

Google Sheets
*************
.. figure:: Images/Google\ Sheet.png
   :align: center

   The Inventory sheet of the Google Spreadsheet.

The Google Sheets contain a database of the current inventory in the warehouse; the orders incoming, dispatched, and shipped; and a back-end integration for the Dashboard that it provides its data to for display. The Sheets are organised as follows:

* *Inventory*: The total number of packages available in the warehouse / detected by the camera to be avaiable (for this implementation).
* *IncomingOrders*: The list of orders received till now during operation / execution.
* *OrdersDispatched*: The list of orders dispatched by the UR5_1; i.e, the number of packages sent to the UR5_2 for shipping.
* *OrdersShipped*: The list of orders sent by the UR5_1 to the UR5_2 for shipping out of the warehouse.
* *Dashboard*: The back-end database the the Dashboard draws its data from. All data in the Dashboard sheet has been pulled from the preceding sheets using cell formulae.

Gmail Integration
*****************
.. figure:: Images/Order\ Email.png
   :align: center

   The email sent to the client once the order has been shipped.

The script integration with the Google sheets mentioned above allow emails to be sent on the following events:

* Orders are dispatched
* Orders are shipped

The client can then see the details of their particular order and also the estimated time of delivery.
