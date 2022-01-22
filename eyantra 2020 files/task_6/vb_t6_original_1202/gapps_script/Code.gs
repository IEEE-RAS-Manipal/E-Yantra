function doGet(e){
  
  var ss = SpreadsheetApp.getActive();

  var sheet = ss.getSheetByName(e.parameter["id"]);

  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

  var lastRow = sheet.getLastRow();

  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();

  for (i in headers){

    // loop through the headers and if a parameter name matches the header name insert the value

    if (headers[i] == "Timestamp")
    {
      val = d.toDateString() + ", " + d.toLocaleTimeString();
    }
    else
    {
      val = e.parameter[headers[i]]; 
    }

    // append data to the last row
    cell.offset(lastRow, col).setValue(val);
    col++;
  }

  // Accessing Dispatched Sheet

  var sh1 = SpreadsheetApp.getActiveSpreadsheet().getSheetByName("OrdersDispatched");

  var lastRow1 = sh1.getLastRow();
  var cols1 = sh1.getDataRange().getValues();
  var data1 = cols1[lastRow1-1][cols1[0].indexOf("Dispatch Status")];
  


  if(data1 == "YES") 
  { 

    sh1.getRange('j1').offset([lastRow1-1],[0]).setValue("YES ");

    var intro_dispatch_msg = "Hello!\n" + "Your Order has been dispatched, contact us if you have any questions. We\nare here to help you.\n\nORDER SUMMARY:\n\n";
    
    var dispatch_msg = "Order Number : " + cols1[lastRow1-1][cols1[0].indexOf("Order ID")] + "\nItem : " + cols1[lastRow1-1][cols1[0].indexOf("Item")] + "\nQuantity : " + cols1[lastRow1-1][cols1[0].indexOf("Dispatch Quantity")] + "\nDispatched Date and Time : " + cols1[lastRow1-1][cols1[0].indexOf("Dispatch Date and Time")] + " +0530" + "\nCity : " + cols1[lastRow1-1][cols1[0].indexOf("City")] + "\nCost : " + cols1[lastRow1-1][cols1[0].indexOf("Cost")]; 

    var dispatch_message = intro_dispatch_msg + dispatch_msg

    MailApp.sendEmail("eyrc.vb.1202@gmail.com", " Your Order is Dispatched! ", dispatch_message);
    MailApp.sendEmail("eyrc.vb.0000@gmail.com", " Your Order is Dispatched! ", dispatch_message);
  }


  // Accessing Shipped Sheet

  var sh = SpreadsheetApp.getActiveSpreadsheet().getSheetByName("OrdersShipped");

  var lastRow = sh.getLastRow();
  var cols = sh.getDataRange().getValues();
  var data = cols[lastRow-1][cols[0].indexOf("Shipped Status")];


  if(data == "YES") 
  { 
    sh.getRange('j1').offset([lastRow-1],[0]).setValue("YES ");

    var intro_ship_msg = "Hello!\n" + "Your Order has been shipped. It will be drone delivered to you in 1 day.\nContact us if you have any questions. We are here to help you.\n\nORDER SUMMARY:\n\n";
    
    var ship_msg = "Order Number : " + cols[lastRow-1][cols[0].indexOf("Order ID")] + "\nItem : " + cols[lastRow-1][cols[0].indexOf("Item")] + "\nQuantity : " + cols[lastRow-1][cols[0].indexOf("Shipped Quantity")] + "\nShipped Date and Time : " + cols[lastRow-1][cols[0].indexOf("Shipped Date and Time")] + " +0530" + "\nCity : " + cols[lastRow-1][cols[0].indexOf("City")] + "\nCost : " + cols[lastRow-1][cols[0].indexOf("Cost")] + "\nEstimated Time of Delivery : " + cols[lastRow-1][cols[0].indexOf("Estimated Time of Delivery")] + " +0530";

    var ship_message = intro_ship_msg + ship_msg

    MailApp.sendEmail("eyrc.vb.1202@gmail.com", " Your Order is Shipped! ", ship_message);
    MailApp.sendEmail("eyrc.vb.0000@gmail.com", " Your Order is Shipped! ", ship_message);

  }

  return ContentService.createTextOutput("success");
 
}