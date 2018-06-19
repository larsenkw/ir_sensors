/*
 * This file includes only the code for operating the HC-05 Bluetooth board with 
 * the Arduino for interfacing with an Android phone app.
 *
 * To run this code with ROS
 * 1) you must have ROS running with 'roscore'
 * 2) open a serial node with the command:
 *   $ rosrun rosserial_python serial_node.py [port name]
 * where [port name] gives the port your Arduino is connected to.
 * The default port is: /dev/ttyACM0
 * Now the Arduino should be publishing to the topic specified in the code below.
 *
 * Node name: serial_node
 * 
 * ---Topics---
 * Publishing: /following_status
 * Message type: std_msgs/Bool (0 = stopped, 1 = following)
 * 
 * Publishing: /BT_connection_status
 * Message type: std_msgs/Bool (0 = disconnected, 1 = connected)
 * 
 * Publishing: /other_commands
 * Message type: std_msgs/String (name of the current command)
 */

// ROS Includes
//#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <ir_sensors/Command.h>
// Bluetooth Includes
#include <SoftwareSerial.h>


// Pins
const int RX_PIN = 2; // Arduino RX pin, Bluetooth TX Pin
const int TX_PIN = 3; // Arduino TX pin, Bluetooth RX Pin, this pin should be connected through a Voltage Divider to output 3.3V
const int BT_CONNECTION_PIN = 4; // For checking connection status of the BT board

// Bluetooth Objects
SoftwareSerial BTSerial(RX_PIN, TX_PIN);

// ROS Objects
// Node
ros::NodeHandle nh;
// Publishers
std_msgs::Time time;
ros::Publisher bt_timestamp("BT_timestamp", &time);
std_msgs::Bool following;
ros::Publisher bt_following("following_status", &following);
std_msgs::Bool connection;
ros::Publisher bt_connection("BT_connection_status", &connection);
ir_sensors::Command command;
ros::Publisher bt_command("bt_command", &command);
// Subscribers
void callback_command_response(const ir_sensors::Command &command_response) {
  if (strcmp(command_response.name, "checkuser")==0) {
    BTSerial.print(command_response.value1);
    BTSerial.print("\n"); // send a newline character so the phone knows it is the end of the response value
  }
  if (strcmp(command_response.name, "checkpassword")==0) {
    BTSerial.print(command_response.value1); // notes whether password was found
    BTSerial.print("\n");
    BTSerial.print(command_response.value2); // displayname of user (if password is correct, blank otherwise)
    BTSerial.print("\n");
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
}
ros::Subscriber<ir_sensors::Command> bt_command_response("bt_command_response", &callback_command_response);

void setup(){
  // ROS Initialization
  nh.initNode();
  nh.advertise(bt_timestamp);
  nh.advertise(bt_following);
  nh.advertise(bt_connection);
  nh.advertise(bt_command);
  nh.subscribe(bt_command_response);

  // Bluetooth Initialization
  BTSerial.begin(38400);
  Serial.begin(57600);
  pinMode(BT_CONNECTION_PIN, INPUT);
  following.data = false;
  connection.data = false;

  pinMode(13, OUTPUT);
}

void loop(){
  // Read Bluetooth Data and Status
  char c;
  String c_string;
  if (BTSerial.available()) {
    c = BTSerial.read();
    //c_string = BTSerial.read();
    //Serial.write(BTSerial.read());
    if (c == '0') {
      stopSignal();
    }
    if (c == '1') {
      followSignal();
    }
    if (c == '2') {
      checkoutCommand();
    }
    if (c == '3') {
      checkuserCommand();
    }
    if (c == '4') {
      checkPasswordCommand();
    }
  }
  connection.data = digitalRead(BT_CONNECTION_PIN);
  bt_connection.publish(&connection);

  nh.spinOnce();
  delay(50);
}
// Command 0
void stopSignal() {
  following.data = false;
  bt_following.publish(&following);
}
// Command 1
void followSignal() {
  following.data = true;
  bt_following.publish(&following);
  
}
// Command 2
void checkoutCommand() {
  command.name = "checkout";
  bt_command.publish(&command);
  BTSerial.print("\nChecking out items.");
}
// Command 3
void checkuserCommand() {
  command.name = "checkuser";
  String value1 = "";
  while (BTSerial.available()) {
    char c = BTSerial.read();
    if (c != '\n') {
      value1 += c;
    }
    else {
      break;
    }
  }
  const char temp[value1.length() + 1];
  value1.toCharArray(temp, value1.length() + 1);
  command.value1 = temp;
  command.value2 = "";
  //Serial.print("Sending command: checkuser, value: " + value1);
  bt_command.publish(&command);
}
// Command 4
void checkPasswordCommand() {
  command.name = "checkpassword";
  String value1 = "";
  String value2 = "";
  // Store first value until newline character
  while (BTSerial.available()) {
    char c = BTSerial.read();
    if (c != '\n') {
      value1 += c;
    }
    else {
      break;
    }
  }
  // Store second value until newline character
  while (BTSerial.available()) {
    char c = BTSerial.read();
    if (c != '\n') {
      value2 += c;
    }
    else {
      break;
    }
  }
  const char temp1[value1.length() + 1];
  const char temp2[value2.length() + 1];
  value1.toCharArray(temp1, value1.length() + 1);
  value2.toCharArray(temp2, value2.length() + 1);
  command.value1 = temp1;
  command.value2 = temp2;
  bt_command.publish(&command);
}

