#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <iostream>
#include <math.h>
using namespace std;


// Create globally accessible node
//ros::NodeHandle nh;

class Server
{
private:
    ros::Subscriber sub_combined_pose;
    ros::Subscriber sub_following_status;
    ros::Subscriber sub_connection_status;
    ros::Subscriber sub_command;
    ros::Subscriber sub_num_sensors;
    ros::Publisher pub_velocity_command;
    ros::NodeHandle nh;
    // Message variables
    std_msgs::Bool following; // indicates whether phone app says to follow
    std_msgs::Bool connection; // indicates whether bluetooth is connected
    std_msgs::String command; // command name
    geometry_msgs::PoseStamped pose; // the human's pose w.r.t. IR sensors
    geometry_msgs::TwistStamped velocity; // robot velocity command
    // Method selection variables
    bool using_ir;
    bool using_camera;
    // Sensor limits
    double ir_d_offset; // m
    double ir_d_max; // m
    double ir_d_min; // m
    double ir_delta; // m
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
        // Grab incoming pose from logic function
        //sub_combined_pose = nh.subscribe<geometry_msgs::PoseStamped>("combined_pose", 10, &Server::poseCallback, this);
        sub_combined_pose = nh.subscribe<geometry_msgs::PoseStamped>("IR_pose", 10, &Server::poseCallback, this);
        // Receive following status
        sub_following_status = nh.subscribe<std_msgs::Bool>("following_status", 10, &Server::followingCallback, this);
        // Receive connection status
        sub_connection_status = nh.subscribe<std_msgs::Bool>("bt_connection_status", 10, &Server::connectionCallback, this);
        // Receive new command
        sub_command = nh.subscribe<std_msgs::String>("other_commands", 10, &Server::commandCallback, this);
        // Receives the number of sensors not maxed out
        sub_num_sensors = nh.subscribe<std_msgs::Int8>("num_sensors", 10, &Server::numSensorsCallback, this);
        // Publishes final velocity command to velocity smoother
        pub_velocity_command = nh.advertise<geometry_msgs::TwistStamped>("vel_cmd_obstacle", 10);

        following.data = false;
        connection.data = false;
        command.data = "";

        using_ir = true;
        using_camera = false;
        ir_d_offset = 0.50;
        ir_d_max = 1.40;
        ir_d_min = 0.20;
        ir_delta = 0.10;
        correcting_direction = 0;
        k_lin_forward = 0.8;
        k_lin_reverse = 1.5;
        k_ang = 10;
        num_sensors = 0;

        //FIXME: Currently assigning ir values to commom values
        d_offset = ir_d_offset;
        d_max = ir_d_max;
        d_min = ir_d_min;
        delta = ir_delta;
    }
    // Update following status when new command received
    void followingCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        following.data = msg->data;
        followCommand();
    }
    // Update connection status when new command received
    void connectionCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        connection.data = msg->data;
        followCommand();
    }
    // Update command status when new command received
    void commandCallback(const std_msgs::String::ConstPtr &msg)
    {
        command.data = msg->data;
        followCommand();
    }

    // Update current pose when received
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        pose = *msg;
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
            pub_velocity_command.publish(velocity);
            ROS_INFO("Stopped, lin.x: %f, ang.z: %f", velocity.twist.linear.x, velocity.twist.angular.z);
        }
    }

    // Sends final velocity command
    void velocityCommand(){
        //----- Linear Velocity -----//
        // Proportional control for velocity
        double distance = sqrt(pow(pose.pose.position.x,2) + pow(pose.pose.position.y,2));
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
        if (pose.pose.position.x == 0){
            velocity.twist.angular.z = 0;
        }
        else {
            velocity.twist.angular.z = k_ang*atan2(pose.pose.position.y,pose.pose.position.x);
        }

        // If distance is out of range, STOP
        if (using_ir){
            // IR Check, sees if the number of sensors maxed out is 6
            if (num_sensors == 0){
                velocity.twist.linear.x = 0;
                velocity.twist.angular.z = 0;
                correcting_direction = 0;
            }
        }
        else{
            // Camera Check
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

        /*
        // Check locations of obstacles, if present block motion in that direction
        // Turning left is position, right is negative (right-hand rule)
        // 1: Front Left, block forward motion and left turning
        if (obstacles[0] == 1){
            if (velocity.twist.linear.x > 0){
                velocity.twist.linear.x = 0;
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
            if (velocity.twist.angular.z > 0){
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
            if (velocity.twist.angular.z < 0){
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
                velocity.twist.linear.x = 0;
            }
            if (velocity.twist.angular.z < 0){
                velocity.twist.angular.z = 0;
            }
        }
        */

        // Publish new velocity command to velocity smoother topic
        pub_velocity_command.publish(velocity);
        ROS_INFO("Following, lin.x: %f, ang.z: %f", velocity.twist.linear.x, velocity.twist.angular.z);
    }
};

int main(int argc, char **argv)
{
    // ROS Node Initialization
    ros::init(argc, argv, "velocity_control");
    Server server;

    ros::spin();

    return 0;
}
