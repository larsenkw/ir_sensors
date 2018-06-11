################################################################################
IR Sensor Package for ROS
################################################################################

6 Jun 2018
There are currently three nodes in this package required to run the demo. One of these nodes is for reading a phone app which should be merged into the Arduino node once the bluetooth hardware arrives, leaving only two nodes to worry about.

Required files:
ir_sensor_node.ino (Arduino node code)
ir_data_process.py (data processing code)
'20-150cm Inverse Plot Data.csv' (data for IR regression function)

================================================================================
Nodes
================================================================================
ir_sensor_node/ir_sensor_node.ino

This is the code that is run on the Arduino and must be uploaded through by using the IDE if not already on the Arduino. Take note of the port that the Arduino is on because you will need to edit that in the '.launch' file if it is different.

ir_data_process.py

This code subscribes to the raw data sent by the Arduino, processes it, then sends it to the controller as a PoseStamped message. The major structure is the 'IRSensors' object which stores the data and functions used to find the average of the past few data points to dampen noise and compute the pose of the person in front.
