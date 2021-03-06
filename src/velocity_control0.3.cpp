/*
 *  This version of the controller works with the Kinect skeleton tracker
 *  messages and requires the gesture_kinect node to run and publish pose
 *  messages. If the skeleton is lost, it should publish a zero pose.
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "body_tracker_msgs/Skeleton.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <sstream>
#include <iostream>
#include <math.h>
#include <tf/transform_listener.h>
#include <boost/bind.hpp>
#include <ctime>
#include <iomanip>
using namespace std;


class Server
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_following_status;
    ros::Subscriber sub_connection_status;
    ros::Subscriber sub_command;
    ros::Subscriber sub_num_sensors;
    ros::Publisher pub_velocity_command;
    ros::Publisher pub_follow_person;
    ros::Publisher pub_obstacle_person;
    tf::TransformListener listener;
    // Message variables
    std_msgs::Bool following; // indicates whether phone app says to follow
    std_msgs::Bool connection; // indicates whether bluetooth is connected
    std_msgs::String command; // command name
    geometry_msgs::PoseStamped ir_pose; // the human's pose w.r.t. IR sensors
    geometry_msgs::PoseStamped cam_pose; // the human's pose w.r.t. camera
    geometry_msgs::PoseStamped ir_pose_robot; // human's pose w.r.t. robot
    geometry_msgs::PoseStamped cam_pose_robot; // human's pose w.r.t. robot
    geometry_msgs::PoseStamped selected_pose; // pose selected by logic function for velocity command
    geometry_msgs::PoseStamped selected_pose_avg; // Calculated using Exponentially Weight Moving Average
    geometry_msgs::PoseStamped selected_pose_avg_prev; // Used in recursive EWMA calculation
    double alpha; // Learning rate parameter for EWMA calculation
    geometry_msgs::TwistStamped velocity; // robot velocity command
    body_tracker_msgs::Skeleton cam_skeleton;
    int body_id;
    // Method selection variables
    bool using_camera;
    bool using_ir;
    // Sensor limits
    double ir_d_offset; // m
    geometry_msgs::PointStamped ir_d_max; // m
    geometry_msgs::PointStamped ir_d_min; // m
    geometry_msgs::PointStamped ir_d_max_robot; // m
    geometry_msgs::PointStamped ir_d_min_robot; // m
    double ir_delta; // m
    double cam_d_offset; // m
    geometry_msgs::PointStamped cam_d_max; // m
    geometry_msgs::PointStamped cam_d_min; // m
    geometry_msgs::PointStamped cam_zero_pos; // m
    geometry_msgs::PointStamped cam_d_max_robot; // m
    geometry_msgs::PointStamped cam_d_min_robot; // m
    geometry_msgs::PointStamped cam_zero_pos_robot; // m
    double cam_delta; // m
    double d_offset; // m
    double d_max; // m
    double d_min; // m
    double delta; // m
    double d_error; // distance between robot and desire point
    // Control parameters
    int correcting_direction; // indicates whether the robot is too far (1) or too close (-1) or stopped (0)
    double k_lin_forward; // linear velocity proportion parameter (for forward motion)
    double k_lin_reverse; // lienar velocity proportion parameter (for reverse motion)
    double k_ang; // angular velocity proportion parameter
    // Stopping criteria variables
    int num_sensors; // number of front IR sensors not maxed out on reading
public:
    Server()
    {
        // Receive following status
        sub_following_status = nh.subscribe<std_msgs::Bool>("following_status", 10, &Server::followingCallback, this);
        // Receive connection status
        sub_connection_status = nh.subscribe<std_msgs::Bool>("connection_status", 10, &Server::connectionCallback, this);
        // Receive new command
        sub_command = nh.subscribe<std_msgs::String>("other_commands", 10, &Server::commandCallback, this);
        // Receives the number of sensors not maxed out
        sub_num_sensors = nh.subscribe<std_msgs::Int8>("num_sensors", 10, &Server::numSensorsCallback, this);
        // Publishes final velocity command to velocity smoother
        pub_velocity_command = nh.advertise<geometry_msgs::Twist>("vel_cmd_obstacle", 10);
        // Publishes the skeleton for the person we are following
        pub_follow_person = nh.advertise<body_tracker_msgs::Skeleton>("follow_person", 10);
        // Publishes the skeleton of the people who are obstacles
        pub_obstacle_person = nh.advertise<body_tracker_msgs::Skeleton>("obstacle_person", 10);

        following.data = false;
        connection.data = false;
        command.data = "";

        using_camera = true;
        using_ir = false;
        ir_d_offset = 0.90;

        // Set min and max points in sensor frames
        ir_d_max.header.frame_id = "IR_frame";
        ir_d_max.point.x = 0.0;
        ir_d_max.point.y = 0.0;
        ir_d_max.point.z = 1.50;
        ir_d_min.header.frame_id = "IR_frame";
        ir_d_min.point.x = 0.0;
        ir_d_min.point.y = 0.0;
        ir_d_min.point.z = 0.20;
        ir_delta = 0.10;
        cam_d_offset = 0.90;
        cam_d_max.header.frame_id = "Cam_frame";
        cam_d_max.point.x = 3.50;
        cam_d_max.point.y = 0.0;
        cam_d_max.point.z = 0.0;
        cam_d_min.header.frame_id = "Cam_frame";
        cam_d_min.point.x = 0.50;
        cam_d_min.point.y = 0.0;
        cam_d_min.point.z = 0.0;
        cam_zero_pos.header.frame_id = "Cam_frame";
        cam_zero_pos.point.x = 0;
        cam_zero_pos.point.y = 0;
        cam_zero_pos.point.z = 0;
        cam_delta = 0.10;

        // Common values
        d_offset = 1.00; // m
        delta = 0.10; // m
        correcting_direction = 0;
        k_lin_forward = 0.8;
        k_lin_reverse = 1.0;
        k_ang = 2;
        num_sensors = 0;
        alpha = 0.7;
        body_id = -1;
    }

    void controlLoop() {
        // Get poses from IR sensors and camera

        // FIXME: print timing messages
        cout << "==============================================================\n";
        cout << "Timing messages\n";
        std::clock_t start = std::clock();
        cout << "pre-get camera: " << 0 << "ms\n";

        // Grabbing pose from /camera_pose topic from Kinect data
        boost::shared_ptr<geometry_msgs::PoseStamped const> sharedPtr_cam;
        sharedPtr_cam = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("camera_pose", ros::Duration(1.0));
        if (sharedPtr_cam != NULL){
            cam_pose = *sharedPtr_cam;
        }
        else {
            // No message received
            cam_pose.pose.position.x = cam_zero_pos.point.x;
            cam_pose.pose.position.z = cam_zero_pos.point.z;
            cout << "No camera message.\n";
        }

        //FIXME: print timing messages
        std::clock_t end = std::clock();
        cout << "post-get camera: " << setprecision(6) << 1000.0 * (end - start) / CLOCKS_PER_SEC << "ms\n";

        // // Grabbing body_tracker/skeleton message instead of pose
        // boost::shared_ptr<body_tracker_msgs::Skeleton const> sharedPtr_cam;
        // sharedPtr_cam = ros::topic::waitForMessage<body_tracker_msgs::Skeleton>("/body_tracker/skeleton", ros::Duration(0.1));
        // if (sharedPtr_cam != NULL) {
        //     cam_skeleton = *sharedPtr_cam;
        //     cout << "Body_id: " << body_id << endl;
        //     cout << "Skeleton_id: " << cam_skeleton.body_id << endl;
        //     if (body_id == -1) {
        //         body_id = cam_skeleton.body_id;
        //     }
        //     if (body_id != cam_skeleton.body_id) {
        //         if (using_ir) { // we lost the peson, but they got close enough to be in the IR range, set new ID when they come back into view
        //             body_id = cam_skeleton.body_id;
        //
        //             cout << "Updating body id\n";
        //         }
        //         else { // this person is an obstacle
        //             pub_obstacle_person.publish(cam_skeleton);
        //
        //             cout << "Sending obstacle skeleton\n";
        //         }
        //     }
        //     else { // the body_id matches
        //         cam_pose.header.frame_id = "Cam_frame";
        //         cam_pose.pose.position.x = cam_skeleton.joint_position_spine_mid.x;
        //         cam_pose.pose.position.y = cam_skeleton.joint_position_spine_mid.y;
        //         cam_pose.pose.position.z = cam_skeleton.joint_position_spine_mid.z;
        //         cam_pose.pose.orientation.x = 0;
        //         cam_pose.pose.orientation.y = 0;
        //         cam_pose.pose.orientation.z = 0;
        //         cam_pose.pose.orientation.w = 1;
        //         pub_follow_person.publish(cam_skeleton);
        //
        //         cout << "Updating following skeleton\n";
        //     }
        // }
        // else {
        //     cam_pose.header.frame_id = "Cam_frame";
        //     cam_pose.pose.position.x = 0;
        //     cam_pose.pose.position.y = 0;
        //     cam_pose.pose.position.z = 0;
        //     cam_pose.pose.orientation.x = 0;
        //     cam_pose.pose.orientation.y = 0;
        //     cam_pose.pose.orientation.z = 0;
        //     cam_pose.pose.orientation.w = 1;
        // }

        //FIXME: print timing messages
        start = std::clock();
        cout << "pre-get ir: " << 0 << "ms\n";

        // Grab pose from IR data
        boost::shared_ptr<geometry_msgs::PoseStamped const> sharedPtr_ir;
        sharedPtr_ir = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("IR_pose", ros::Duration(1.0));

        if (sharedPtr_ir != NULL){
            ir_pose = *sharedPtr_ir;
        }
        else {
            cout << "No IR message.\n";
        }

        //FIXME: print timing messages
        end = std::clock();
        cout << "post-get ir: " << setprecision(6) << 1000.0 * (end - start) / CLOCKS_PER_SEC << "ms\n";

        // FIXME: test raw data from camera and IR
        cout << "body_id: " << body_id << endl;
        cout << "Camera raw: (" << cam_pose.pose.position.x << "," << cam_pose.pose.position.y << "," << cam_pose.pose.position.z << ")\n";
        cout << "IR raw: (" << ir_pose.pose.position.x << "," << ir_pose.pose.position.y << "," << ir_pose.pose.position.z << ")\n";

        // Transform poses and points into current robot frame
        transformPosesAndPoints();

        // Perform logic to determine sensor to use
        selectPose();

        // Run follow command function to process newly selected pose
        followCommand();
    }

    // Transform all sensor values into the robot frame
    void transformPosesAndPoints(){

        //FIXME: print timing messages
        std::clock_t start = std::clock();
        cout << "pre-cam transform: " << 0 << "ms\n";

        // Transform Camera Frame
        ros::Time now = cam_pose.header.stamp;
        try {
            listener.waitForTransform("/base_footprint", "/Cam_frame", now, ros::Duration(1.0));
            // Transform Poses
            listener.transformPose("/base_footprint", cam_pose, cam_pose_robot);
            // Transform Points
            listener.transformPoint("/base_footprint", cam_d_max, cam_d_max_robot);
            listener.transformPoint("/base_footprint", cam_d_min, cam_d_min_robot);
            listener.transformPoint("/base_footprint", cam_zero_pos, cam_zero_pos_robot);

            // FIXME: print out camera pose in robot frame
            cout << "Cam: (" << cam_pose_robot.pose.position.x << "," << cam_pose_robot.pose.position.y << ")\n";
        }
        catch(tf::TransformException& ex) {
            ROS_ERROR("Cam Transform Exception: %s", ex.what());
            // // FIXME: test raw data from camera and IR
            // cout << "Camera after breaking: (" << cam_pose.pose.position.x << "," << cam_pose.pose.position.y << "," << cam_pose.pose.position.z << ")\n";
            // cout << "Camera Robot after breaking: (" << cam_pose_robot.pose.position.x << "," << cam_pose_robot.pose.position.y << "," << cam_pose_robot.pose.position.z << ")\n";
            // cout << "Max: (" << cam_d_max.point.x << "," << cam_d_max.point.y << "," << cam_d_max.point.z << ")\n";
            // cout << "Max Robot: (" << cam_d_max_robot.point.x << "," << cam_d_max_robot.point.y << "," << cam_d_max_robot.point.z << ")\n";
            // cout << "Min: (" << cam_d_min.point.x << "," << cam_d_min.point.y << "," << cam_d_min.point.z << ")\n";
            // cout << "Min Robot: (" << cam_d_min_robot.point.x << "," << cam_d_min_robot.point.y << "," << cam_d_min_robot.point.z << ")\n";
            // cout << "Zero: (" << cam_zero_pos.point.x << "," << cam_zero_pos.point.y << "," << cam_zero_pos.point.z << ")\n";
            // cout << "Zero Robot: (" << cam_zero_pos_robot.point.x << "," << cam_zero_pos_robot.point.y << "," << cam_zero_pos_robot.point.z << ")\n";

            //cam_d_max_robot.pose.position.x = cam_zero_pos_robot.position.x;
        }

        //FIXME: print timing messages
        std::clock_t end = std::clock();
        cout << "post-cam transform: " << setprecision(6) << 1000.0 * (end - start) / CLOCKS_PER_SEC << "ms\n";

        //FIXME: print timing messages
        start = std::clock();
        cout << "pre-ir transform: " << 0 << "ms\n";

        // Transform IR Frame
        now = ros::Time(0);
        try {
            listener.waitForTransform("/base_footprint", "/IR_frame", now, ros::Duration(1.0));
            // Transform Poses
            listener.transformPose("/base_footprint", ir_pose, ir_pose_robot);
            // Transform Points
            listener.transformPoint("/base_footprint", ir_d_max, ir_d_max_robot);
            listener.transformPoint("/base_footprint", ir_d_min, ir_d_min_robot);

            // FIXME: print out ir pose in robot frame
            cout << "IR: (" << ir_pose_robot.pose.position.x << "," << ir_pose_robot.pose.position.y << ")\n";
        }
        catch(tf::TransformException& ex) {
            ROS_ERROR("IR Transform Exception: %s", ex.what());
        }

        //FIXME: print timing messages
        end = std::clock();
        cout << "post-ir transform: " << setprecision(6) << 1000.0 * (end - start) / CLOCKS_PER_SEC << "ms\n";
    }

    // Update following status when new command received
    void followingCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        following.data = msg->data;
        //followCommand();
    }
    // Update connection status when new command received
    void connectionCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        connection.data = msg->data;
        //followCommand();
    }
    // Update command status when new command received
    void commandCallback(const std_msgs::String::ConstPtr &msg)
    {
        command.data = msg->data;
        //followCommand();
    }

    // Update current pose when received
    void selectPose()
    {
        // // FIXME: Using only camera values for testing
        // // Check if camera pose is valid, if so, use camera
        // if (cam_pose_robot.pose.position.x != cam_zero_pos_robot.point.x) {
        //     if ((cam_pose_robot.pose.position.x <= cam_d_max_robot.point.x) and
        //         (cam_pose_robot.pose.position.x >= cam_d_min_robot.point.x)) {
        //         selected_pose = cam_pose_robot;
        //         d_max = cam_d_max_robot.point.x;
        //         d_min = cam_d_min_robot.point.x;
        //         using_camera = true;
        //         using_ir = false;
        //         ROS_INFO("Camera selected");
        //     }
        // }
        // // Else search for person.
        // else {
        //     using_camera = false;
        //     using_ir = false;
        //     findPerson();
        // }
        // // FIXME: Using only camera values for testing


        // Check if camera pose is valid, if so, use camera
        if (cam_pose_robot.pose.position.x != cam_zero_pos_robot.point.x) {
            if ((cam_pose_robot.pose.position.x <= cam_d_max_robot.point.x) and
                (cam_pose_robot.pose.position.x >= cam_d_min_robot.point.x)) {
                selected_pose = cam_pose_robot;
                d_max = cam_d_max_robot.point.x;
                d_min = cam_d_min_robot.point.x;
                using_camera = true;
                using_ir = false;
                ROS_INFO("Camera selected");
            }
        }
        // If camera pose is not value and IR pose is valid, use IR
        // Checks IR values are within range and at least one sensor is not maxed out
        else if ((ir_pose_robot.pose.position.x <= ir_d_max_robot.point.x) and
                 (ir_pose_robot.pose.position.x >= ir_d_min_robot.point.x) and
                 (num_sensors != 0)) {
            selected_pose = ir_pose_robot;
            d_max = ir_d_max_robot.point.x;
            d_min = ir_d_min_robot.point.x;
            using_camera = false;
            using_ir = true;
            ROS_INFO("IR selected");
        }
        // Else search for person.
        else {
            using_camera = false;
            using_ir = false;
            findPerson();
        }
    }

    // Function for finding the person again (to be implemented in the future)
    void findPerson() {
        ROS_INFO("Finding person...");
        // Right now this function just stops the robot
        following.data = false;
        followCommand();
    }

    // Update number of sensors not maxed out
    void numSensorsCallback(const std_msgs::Int8::ConstPtr &msg)
    {
        num_sensors = msg->data;
    }

    // Handles following command, turns off if not following or connected
    void followCommand()
    {
        if (following.data and connection.data){
            velocityCommand();
        }
        else {
            velocity.twist.linear.x = 0;
            velocity.twist.angular.z = 0;
            pub_velocity_command.publish(velocity.twist);
            //ROS_INFO("Stopped, lin.x: %f, ang.z: %f, camera: %d, ir: %d", velocity.twist.linear.x, velocity.twist.angular.z, using_camera, using_ir);
        }
    }

    // Sends final velocity command
    void velocityCommand(){
        // FIXME: print out selected pose for comparison
        cout << "Selected: (" << selected_pose.pose.position.x << "," << selected_pose.pose.position.y << ")\n";

        // Use Exponentially Weighted Moving Average for pose to smooth out changes
        if (selected_pose.header.seq <= 1) {
            selected_pose_avg = selected_pose;
            selected_pose_avg_prev = selected_pose_avg;
        }
        else {
            selected_pose_avg.header = selected_pose.header;
            selected_pose_avg.pose.position.x = alpha*selected_pose.pose.position.x + (1 - alpha)*selected_pose_avg_prev.pose.position.x;
            selected_pose_avg.pose.position.y = alpha*selected_pose.pose.position.y + (1 - alpha)*selected_pose_avg_prev.pose.position.y;
            selected_pose_avg.pose.position.z = alpha*selected_pose.pose.position.x + (1 - alpha)*selected_pose_avg_prev.pose.position.x;
            selected_pose_avg.pose.orientation.x = alpha*selected_pose.pose.orientation.x + (1 - alpha)*selected_pose_avg_prev.pose.orientation.x;
            selected_pose_avg.pose.orientation.y = alpha*selected_pose.pose.orientation.y + (1 - alpha)*selected_pose_avg_prev.pose.orientation.y;
            selected_pose_avg.pose.orientation.z = alpha*selected_pose.pose.orientation.z + (1 - alpha)*selected_pose_avg_prev.pose.orientation.z;
            selected_pose_avg.pose.orientation.w = alpha*selected_pose.pose.orientation.w + (1 - alpha)*selected_pose_avg_prev.pose.orientation.w;
            selected_pose_avg_prev = selected_pose_avg;
        }

        // Make the EWMA Pose the current selected pose
        selected_pose = selected_pose_avg;

        //----- Linear Velocity -----//
        // Proportional control for velocity
        double distance = sqrt(pow(selected_pose.pose.position.x,2) + pow(selected_pose.pose.position.y,2));
        if (distance <= d_max){
            d_error = distance - d_offset;
            // If distance is outside d_offset +/- delta then move until you get to d_offset
            if (correcting_direction == 0){
                if (d_error > delta){
                    // Need to move forward
                    correcting_direction = 1;
                }
                else if (d_error < -delta){
                    // Need to move backwards
                    correcting_direction = -1;
                }
            }

            if (correcting_direction == 1){
                // Currently moving forward
                if (d_error <= 0){
                    velocity.twist.linear.x = 0.0;
                    correcting_direction = 0;
                }
                else {
                    velocity.twist.linear.x = k_lin_forward*d_error;
                }
            }
            else if (correcting_direction == -1){
                // Currently movingn backward
                if (d_error >= 0){
                    velocity.twist.linear.x = 0.0;
                    correcting_direction = 0;
                }
                else {
                    velocity.twist.linear.x = k_lin_reverse*d_error;
                }
            }
        }

        //----- Angular Velocity -----//
        // Angular velocity is proportional to the number of sensors that are maxed out
        if (selected_pose.pose.position.x == 0){
            velocity.twist.angular.z = 0;
        }
        else {
            if (correcting_direction == 0) {
                if (abs(atan2(selected_pose.pose.position.y,selected_pose.pose.position.x)) < 0.01) {
                    velocity.twist.angular.z = 0;
                }
            }
            else {
                velocity.twist.angular.z = k_ang*atan2(selected_pose.pose.position.y,selected_pose.pose.position.x);
            }
        }

        // Send velocity to obstacle checker
        obstacleCheck();
    }

    // Take calculated velocity and check for obstacles, then publish command
    void obstacleCheck()
    {
        // Get obstacle param
        std::vector<int> obstacles;
        nh.getParam("obstacles", obstacles);

        // Check locations of obstacles, if present block motion in that direction
        // Turning left is position, right is negative (right-hand rule)
        // 1: Front Left, block forward motion and left turning
        if (obstacles[0] == 1){
            if (velocity.twist.linear.x > 0){
                velocity.twist.linear.x *= 0.1;
            }
            if (velocity.twist.angular.z > 0){
                velocity.twist.angular.z = 0;
            }
        }
        // 2: Left, block left turning
        if (obstacles[1] == 1){
            if (velocity.twist.angular.z > 0){
                velocity.twist.angular.z = 0;
            }
        }
        // 3: Back Left, block backward motion and left turning
        if (obstacles[2] == 1){
            if (velocity.twist.linear.x < 0){
                velocity.twist.linear.x = 0;
            }
            if (velocity.twist.angular.z < 0){
                velocity.twist.angular.z = 0;
            }
        }
        // 4: Back, block backward motion
        if (obstacles[3] == 1){
            if (velocity.twist.linear.x < 0){
                velocity.twist.linear.x = 0;
            }
        }
        // 5: Back Right, block backward motion and right turning
        if (obstacles[4] == 1){
            if (velocity.twist.linear.x < 0){
                velocity.twist.linear.x = 0;
            }
            if (velocity.twist.angular.z > 0){
                velocity.twist.angular.z = 0;
            }
        }
        // 6: Right, block right turning
        if (obstacles[5] == 1){
            if (velocity.twist.angular.z < 0){
                velocity.twist.angular.z = 0;
            }
        }
        // 7: Front Right, block forward motion and right turning
        if (obstacles[6] == 1){
            if (velocity.twist.linear.x > 0){
                velocity.twist.linear.x *= 0.1;
            }
            if (velocity.twist.angular.z < 0){
                velocity.twist.angular.z = 0;
            }
        }

        // Publish new velocity command to velocity smoother topic
        pub_velocity_command.publish(velocity.twist);
        //ROS_INFO("Following, lin.x: %f, ang.z: %f, camera: %d, ir: %d", velocity.twist.linear.x, velocity.twist.angular.z, using_camera, using_ir);
    }
};

int main(int argc, char **argv)
{
    // ROS Node Initialization
    ros::init(argc, argv, "velocity_control");
    Server server;

    ros::Rate loop_rate(60);

    while(ros::ok()) {
        server.controlLoop();
        ros::spinOnce();
        //loop_rate.sleep();
    }
    //ros::spin();

    return 0;
}
