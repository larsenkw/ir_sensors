/*
 * This code is for reading distance using a Sharp 0A41SK0F IR Distance sensor
 * Unit measures between 4 and 30 cm
 *
 * To run this code with ROS
 * 1) you must have ROS running with 'roscore'
 * 2) open a serial node with the command:
 *   $ rosrun rosserial_python serial_node.py [port name]
 * where [port name] gives the port your Arduino is connected to.
 * The default port is: /dev/ttyACM0
 * Now the Arduino should be publishing to the topic specified in the code below.
 *
 * The node name is: serial_node
 * Publishing to topic: ir_raw_data
 * The message is of type: std_msgs/Int16MultiArray
 */

#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Time.h>


// Pins
const int SENSOR1 = 0;
const int SENSOR2 = 1;
const int SENSOR3 = 2;
const int SENSOR4 = 3;
const int SENSOR5 = 4;
const int SENSOR6 = 5;
const int IR_SENSORS[6] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6};
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

void setup(){
  nh.initNode();
  int16_array.data_length = 6; // length of array
  nh.advertise(ir_sensors);
  nh.advertise(ir_sensors_timestamp);
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
  nh.spinOnce();
  delay(50);
}
