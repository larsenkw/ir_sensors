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
 * Message type: sensor_msgs/LaserScan
 * 
 * Publishing: /following_status
 * Message type: std_msgs/Bool (0 = stopped, 1 = following)
 * 
 * Publishing: /bt_connection_status
 * Message type: std_msgs/Bool (0 = disconnected, 1 = connected)
 * 
 * Publishing: /bt_command
 * Message type: std_msgs/String (name of the current command)
 */

// ROS Includes
//#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
// Bluetooth Includes
#include <SoftwareSerial.h>


// Pins
const int SENSOR1 = A0;
const int SENSOR2 = A1;
const int SENSOR3 = A2;
const int SENSOR4 = A3;
const int SENSOR5 = A4;
const int SENSOR6 = A5;
const int IR_SENSORS[6] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6};
// Arduino UNO
//const int RX_PIN = 2; // Arduino RX pin, Bluetooth TX Pin
//const int TX_PIN = 3; // Arduino TX pin, Bluetooth RX Pin, this pin should be connected through a Voltage Divider to output 3.3V
// Arduino MEGA (pin 2 does not support change interrupts)
const int RX_PIN = 10;
const int TX_PIN = 11;
const int BT_CONNECTION_PIN = 4; // For checking connection status of the BT board

const int num_sensors = 6;

// Other parameters
//float data[6];

// ROS Objects
ros::NodeHandle nh;
sensor_msgs::LaserScan ir_scan;
ros::Publisher ir_raw_data("ir_raw_data", &ir_scan);
std_msgs::Bool following;
ros::Publisher bt_following("following_status", &following);
std_msgs::Bool connection;
ros::Publisher bt_connection("bt_connection_status", &connection);
std_msgs::String command;
ros::Publisher bt_command("bt_command", &command);

// Bluetooth Objects
SoftwareSerial BTSerial(RX_PIN, TX_PIN);

void setup(){
  // ROS Initialization
  nh.initNode();
  nh.advertise(ir_raw_data);
  nh.advertise(bt_following);
  nh.advertise(bt_connection);
  nh.advertise(bt_command);
  // rosserial default baud rate = 57600
  Serial.begin(57600);
  
  // Bluetooth Initialization
  BTSerial.begin(38400);
  pinMode(BT_CONNECTION_PIN, INPUT);
  following.data = false;
  connection.data = false;

//  Serial.println(sizeof(nh));
//  Serial.println(sizeof(ir_scan));
//  Serial.println(sizeof(following));
//  Serial.println(sizeof(connection));
//  Serial.println(sizeof(command));
}

void loop(){
  // Gather Data
  for (int i = 0; i < num_sensors; ++i){
    ir_scan.ranges[i] = analogRead(IR_SENSORS[i]);
  }
  
  // Write and send message
  //ir_scan.ranges = data;
  ir_scan.header.stamp = nh.now();
  ir_scan.header.frame_id = "IR Range Data";
  ir_raw_data.publish(&ir_scan);

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
  bt_command.publish(&command);
  BTSerial.print("\nChecking out items.");
}

