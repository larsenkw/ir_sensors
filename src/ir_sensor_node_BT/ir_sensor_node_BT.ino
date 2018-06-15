/*
 * This code is for reading distance using a Sharp 0A41SK0F IR Distance sensor
 * Unit measures between 4 and 30 cm
 * This file now includes code for operating the HC-05 Bluetooth board with 
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
 * Publishing: /ir_raw_data
 * Message type: std_msgs/Int16MultiArray
 * 
 * Publishing: /ir_raw_data_timestamp
 * Message type: std_msgs/Time
 * 
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
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
// Bluetooth Includes
#include <SoftwareSerial.h>


// Pins
const int SENSOR1 = 0;
const int SENSOR2 = 1;
const int SENSOR3 = 2;
const int SENSOR4 = 3;
const int SENSOR5 = 4;
const int SENSOR6 = 5;
const int IR_SENSORS[6] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6};
const int RX_PIN = 2; // Arduino RX pin, Bluetooth TX Pin
const int TX_PIN = 3; // Arduino TX pin, Bluetooth RX Pin, this pin should be connected through a Voltage Divider to output 3.3V
const int BT_CONNECTION_PIN = 4; // For checking connection status of the BT board

int i = 0;
int num_sensors = 6;

// Other parameters
int16_t data[6];

// ROS Objects
ros::NodeHandle nh;
std_msgs::Int16MultiArray int16_array;
ros::Publisher ir_sensors("ir_raw_data", &int16_array);
std_msgs::Time time;
ros::Publisher ir_sensors_timestamp("ir_raw_data_timestamp", &time);
std_msgs::Bool following;
ros::Publisher bt_following("following_status", &following);
std_msgs::Bool connection;
ros::Publisher bt_connection("BT_connection_status", &connection);
std_msgs::String command;
ros::Publisher bt_commands("other_commands", &command);

// Bluetooth Objects
SoftwareSerial BTSerial(RX_PIN, TX_PIN);

void setup(){
  // rosserial baud rate = 57600
  Serial.begin(57600);
  
  // ROS Initialization
  nh.initNode();
  int16_array.data_length = 6; // length of array
  nh.advertise(ir_sensors);
  nh.advertise(ir_sensors_timestamp);
  nh.advertise(bt_following);
  nh.advertise(bt_connection);
  nh.advertise(bt_commands);

  // Bluetooth Initialization
  BTSerial.begin(38400);
  pinMode(BT_CONNECTION_PIN, INPUT);
  following.data = false;
  connection.data = false;
}

void loop(){
  // Gather Data
  for (i = 0; i < num_sensors; ++i){
    data[i] = analogRead(IR_SENSORS[i]);
  }
  
  // Write and send message 
  int16_array.data = data;
  ir_sensors.publish(&int16_array);
  time.data = nh.now();
  ir_sensors_timestamp.publish(&time);

  // Read Bluetooth Data and Status
  char c;
  if (BTSerial.available()) {
    c = BTSerial.read();
    if (c == '0') {
      stopSignal();
    }
    if (c == '1') {
      followSignal();
    }
    if (c == '2') {
      checkoutCommand();
    }
  }
  connection.data = digitalRead(BT_CONNECTION_PIN);
  bt_connection.publish(&connection);

  nh.spinOnce();
  delay(50);
}

void stopSignal() {
  following.data = false;
  bt_following.publish(&following);
}

void followSignal() {
  following.data = true;
  bt_following.publish(&following);
  
}

void checkoutCommand() {
  command.data = "checkout";
  bt_commands.publish(&command);
  BTSerial.print("\nChecking out items.");
}

