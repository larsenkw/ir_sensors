#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"
using namespace std;

class Server
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_joy_command;
    ros::Publisher pub_joy_follow;
    ros::Publisher pub_joy_connection;
    // Messages
    std_msgs::Bool following;
    std_msgs::Bool prev_following;
    std_msgs::Bool connection;
    std_msgs::Bool prev_connection;
    // Controller type 'X' or 'D'
    char configuration;
public:
    Server()
    {
        // Receive joystick message
        sub_joy_command = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Server::joyCallback, this);
        // Send following signal
        pub_joy_follow = nh.advertise<std_msgs::Bool>("joy_following_status", 10);
        pub_joy_connection = nh.advertise<std_msgs::Bool>("joy_connection_status", 10);
        // Default to type 'X'
        configuration = 'X';
        following.data = false;
        prev_following.data = false;
        connection.data = false;
        prev_connection.data = false;
        pub_joy_follow.publish(following);
        pub_joy_connection.publish(connection);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
    {
        // Check controller type, 'X' has 8 axes, 'D' has 6 axes
        if (msg->axes.size() == 8) {
            configuration = 'X';
        }
        else if (msg->axes.size() == 6) {
            configuration = 'D';
        }
        else {
            ROS_ERROR("Unrecognized controller configuration. Should be using Logitech F710 with front switch on 'X' or 'D'.");
        }

        if (configuration == 'X') {
            connection.data = true;
            pub_joy_connection.publish(connection);
            // 'A' button is element 0, 'B' button is element 1
            if (msg->buttons[0] == 1) {
                following.data = true;
                pub_joy_follow.publish(following);
            }
            if (msg->buttons[1] == 1) {
                following.data = false;
                pub_joy_follow.publish(following);
            }
        }
        else if (configuration == 'D') {
            connection.data = true;
            pub_joy_connection.publish(connection);
            // 'A' button is element 1, 'B' button is element 2
            if (msg->buttons[1] == 1) {
                following.data = true;
                pub_joy_follow.publish(following);
            }
            if (msg->buttons[2] == 1) {
                following.data = false;
                pub_joy_follow.publish(following);
            }
        }
        else {
            // Unrecognized configuration
            following.data = false;
            connection.data = false;
            pub_joy_connection.publish(connection);
            pub_joy_follow.publish(following);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_app");
    Server server;
    ros::spin();

    return 0;
}
