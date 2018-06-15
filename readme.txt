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

================================================================================
Adding Custom Messages to Arduino Library
================================================================================
Make message files ([message_file].msg) as outlined in the ROS tutorials and place them in the 'msg' directory of your package.

Go to the top level of your Catkin workspace and perform the make command.

$ catkin_make

This should generate the appropriate message files required. Then remove the current 'ros_lib' folder from the Arduino libraries folder (this is found where your sketchbook folder is, typically ~/sketchbook/libraries). You have to remove all files in the 'ros_lib' folder.

$ rm -r ~/sketchbook/libraries/ros_lib

Then remake the folder with the following command.

$ rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries

A new 'ros_lib' folder will appear which should contain the messages you created in your package. Navigate to the 'ros_lib' folder and make sure you package name is listed there when using 'ls'.

================================================================================
Arduino Troubleshooting
================================================================================
One of the errors I have often received is:

[ERROR] [WallTime: 1529003151.564960] Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino

You can try restarting the Arduino by unplugging and replugging. You can also restart the computer, that has fixed the issue sometimes. One other thing to check is that the Arduino Baud rate is set to match the ROS one. ROS will tell you what is it connecting to and how fast.

[INFO] [WallTime: 1529003134.460763] Connecting to /dev/ttyACM0 at 57600 baud

Make sure the Arduino baud rate matches. You can force it to be a certain value by starting the serial connection with:

Serial.begin(57600)

Then is should be able to sync.

If you get the error "Permission denied: '/dev/ttyACM0'", one possible fix is to change the permissions of that port using:

$ sudo chmod 666 /dev/ttyACM0


