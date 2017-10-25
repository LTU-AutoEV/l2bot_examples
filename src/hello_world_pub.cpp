#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "hello_world_pub");

    // Print messages to ROS
    ROS_INFO_STREAM("hello_world_pub is running!");

    // Every node must a NodeHandle
    // It is the main communication point
    ros::NodeHandle nh{"~"};

    // The publisher object
    // Try changing the published topic to "hello_world" (no forward slash)
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello_world", 10);

    // How often to publish?
    ros::Rate loop_rate(10);

    // The publish loop
    while (ros::ok())
    {
        // Create the message object
        std_msgs::String msg;

        // Set the data
        msg.data = "Hello world!";

        // Publish the message
        pub.publish(msg);

        // wait for loop_rate
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
