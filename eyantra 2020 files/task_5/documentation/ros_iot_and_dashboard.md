## Communication between ROS and IoT:

A bridge has been set up to push and pull data through IoT and ROS through a custom script. The embedded script has provisions for receiving goals through a self-defined action message which the client sends.

This goal is then processed and sent to the spreadsheet using the python library requests. The published orders are then received by the Bridge and are published over a known ROS topic.

## Spreadsheet:

### Dashboard:
The dashboard sheet tab's first row consists of only inscribed formulae. 
To synchronize values from other sheet tabs, the following formula is added:

		=IMPORTRANGE("Spreadsheet Link","Spreadsheet Name!Column2:Column10") #2 and 10 are start and end indices
 
For calculating the total time taken, a check is made to ensure that the arithmetic is not done on blank cells. Then a simple subtraction is made between dates:

		=IF(ISBLANK(N2),"",(N2-L2)*86400) # The index is incremented for each subsequent row.

### Script Editor:

Provisions have been made to embed the current timestamp on the subsequent column's existence for each push of data.

A check is done with the name match of "Dispatched" and "Shipped" for mailing alerts to the mentioned address if the tabs mentioned above enter a true value.  

## Dashboard:

The Dashboard Spreadsheet is linked to this hosted website by embedding the JSON Endpoint URL in an AJAX Request.
The synchronization of the sheet updation on the website is a feature of the above protocol.

Leaflet is used to attach a dynamic interactive map, with provisions to embed markers based on the pushed package coordinates. 
The markers change colours based on their Dispatched and Shipped Status. Once clicked, the markers show the details of the package and the location of the shipment.

Dynamic bar graphs have been included with the help of Google Charts. These bars change colours based on the priority of the package that has been shipped.
The final static graphs indicate the time taken for each package to be initially ordered and finally shipped to the desired location.





