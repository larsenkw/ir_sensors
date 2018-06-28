################################################################################
IR Sensor Package for ROS
################################################################################

6 Jun 2018
There are currently three nodes in this package required to run the demo. One of these nodes is for reading a phone app which should be merged into the Arduino node once the bluetooth hardware arrives, leaving only two nodes to worry about.

Required files:
turtlebot_camera_ir_follow.launch
rosserial_python package for serial_node.py (serial processing node for the Arduino)
ir_sensor_node_BT_laserscan.ino (Arduino node code)
ir_data_process.py (data processing code)
'20-150cm Inverse Plot Data.csv' (data for IR regression function)
IR_Cam_transforms.launch (transforms between the IR/Camera and robot frames)
connection_handler.py (allows control with App or Logitech joystick)
joystick_app_replacement_node.cpp (for sending joystick commands to follow or stop)
velocity_control.cpp (for calculating velocity from sensor data)
vel_smoother.py (my velocity smoother, use turtlebot's built-in one first: velocity_smoother.launch.xml)

================================================================================
Launch Files
================================================================================
Main Launch File:
turtlebot_camera_ir_follow.launch

This launch file is the one that runs the full system. It starts up turtlebot and runs all the other necessary nodes to get the turtlebot following with the IR sensors and the camera.

Transforms Launch File:
IR_Cam_transforms.launch

This launch file begins a 'tf' node and defines the transforms between the /IR_frame and the /Cam_frame to put the IR and camera data into the robot /base_footprint frame.

velocity_smoother.launch.xml

Turtlebot's built-in velocity smoother. Send your velocity commands to "/teleop_velocity_smoother/raw_cmd_vel" or remap your output to that topic in the launch file (this has already been done in 'turtlebot_camera_ir_follow.launch').

================================================================================
Nodes
================================================================================
connection_handler.py

Manages the signals sent to the App and the Logitech controller. If the joystick is available, all it takes is pressing a button or moving an axis to let the program know the controller is connected. There is currently no method of determining if the joystick is no longer connected, so if you unplug it you will need to restart the system to have the connection return to 'false'. To tell the robot to follow press the 'A' button (green). To tell the robot to stop use the 'B' button (red). The Android App establishes a connection and publishes 'true' once the phone is connected. When the phone is disconnected it will publish 'false' (as long as the Logitech controller has not been previously connected). The user can then send follow or stop commands from the phone. This node then passes the connection and following values to the velocity command node to determine whether to stop or calculate the velocity from data.

serial_node.py,
ir_sensor_node_BT_laserscan/ir_sensor_node_BT_laserscan.ino

This is the code that is run on the Arduino and must be uploaded by using the IDE if not already on the Arduino. Take note of the port that the Arduino is on because you will need to edit that in the '.launch' file if it is different. It is the 'arg' with name 'port'. The default value is '/dev/ttyACM0' and should be changed if the Arduino is on another port. The Arduino should be plugged in first to ensure it is in this location.

joy_node

Node from external package 'joy' which reads the commands from the Logitech controller.

joystick_app_replacement_node

This node interprets the commands from the Logitech controller passed on by 'joy_node' and determines whether the system should be following or stopped and passes these values on to the connection_handler node.

ir_data_process.py

This code subscribes to the raw data sent by the Arduino, processes it, then sends it to the controller as a PoseStamped message. The major structure is the 'IRSensors' object which stores the data and functions used to find the average of the past few data points to dampen noise and compute the pose of the person in front.

velocity_control

This node reads the pose information from the IR sensors and the camera and then calculates the velocity of the robot in the robot frame.

vel_smoother.py

This node takes the raw velocity command and smooths the velocity based on the current velocity and a few set parameters to keep the robot from accelerating too fast.

================================================================================
Manual Setup Procedure
================================================================================
This method is for starting up each node manually if you want to debug or do not want to use the turtlebot_camera_ir_follow.launch file for some reason.

Source the workspace setup.bash file for each new tab

$ ~/[workspace_name]/devel/setup.bash
i.e.
$ ~/catkin_ws/devel/setup.bash

Start ROS

$ roscore

Start Turtlebot

$ roslaunch turtlebot_bringup minimal.launch

Begin the serial node for the Arduino

$ rosrun rosserial_python serial_node.py /dev/ttyACM0

Start connection handler

$ rosrun ir_sensors connection_handler.py

Start up joystick

$ joystick_app_replacement.launch
OR
$ rosrun joy joy_node
$ rosrun ir_sensors joystick_app_replacement_node

Start processing IR data

$ rosrun ir_sensors ir_data_process.py

Start processing Camera data

$ rosrun astra_body_tracker astra_body_tracker_node
$ rosrun python_follower gesture_kinect.py

Start velocity controller and smoother

$ rosrun ir_sensors velocity_control
$ rosrun ir_sensors vel_smoother.py

Then begin by connecting to the robot with the App or Logitech controller, stand in front, then send the follow command.

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
Arduino Troubleshooting ================================================================================
----- ROS libraries not recognized
If you get an error saying '<ros.h>' cannot be found then make sure you have the preferences set to point to your sketchbook folder which contains the 'libraries' folder. Typically this should be in '~/sketchbook', so that it can find the files in '~/sketchbook/libraries/ros_lib'.

----- Sync Issues
One of the errors I have often received is:
[ERROR] [WallTime: 1529003151.564960] Unable to sync with device; possible link
problem or link software version mismatch such as hydro rosserial_python with
groovy Arduino

You can try restarting the Arduino by unplugging and replugging. You can also
restart the computer, that has fixed the issue sometimes. One other thing to
check is that the Arduino Baud rate is set to match the ROS one. ROS will tell
you what it is connecting to and how fast.
[INFO] [WallTime: 1529003134.460763] Connecting to /dev/ttyACM0 at 57600 baud

Make sure the Arduino baud rate matches. You can force it to be a certain value
by starting the serial connection with:

Serial.begin(57600)

Then is should be able to sync.

If you get the error "Permission denied: '/dev/ttyACM0'", one possible fix is to
change the permissions of that port using:

$ sudo chmod 666 /dev/ttyACM0

You may also need to add yourself to the 'dialout' group with the following command:

$ [user@machine ~]$ sudo usermod -a -G dialout [user]

Where you type your username instead of [user].

----- Using Array ROS messages
One of the errors I kept getting was:
[ERROR] [WallTime: 1344388188.090867] Lost sync with device, restarting...

One of the particular reasons for this was from using arrays inappropriately in the Arduino code. There are some limitations to using rosserial_arduino which are listed at: http://wiki.ros.org/rosserial/Overview/Limitations. When creating the '.h' files for the messages in the /ros_lib folder found in the Arduino libraries folder (it should be located in the sketchbook folder which is found under preferences, should be ~/sketchbook), these '.h' files take messages that have arrays and change them slightly. Since there is no way for the Arduino to know how long the array is without a null character like with char arrays, it makes a new variable named [array_name]_length where [array_name] is the name of the array as defined in the message. The array itself is also now a pointer which you need to assign to an existing constant length array you have created. For example, if you want to create an array of length 6 you would first create an array and then assign the pointer in the message to that array.

i.e.
int16_t data[6];
[array_name]_length = 6;
[array_name] = data;

Record your values and assign them to the elements in 'data' and then assign [array_name] to 'data' and then publish the message.
Also Assigning the length must be done within the 'setup()' function and not before.
