// ROS and messages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


// L2Bot Controller Topic
#define TWIST_PUB "/rb_drive/rb_drive/twist_cmd"

int main(int argc, char** argv)
{

    // Initialize ROS
    ros::init(argc, argv, "go_fwd");
    ros::NodeHandle nh;

    // Publish
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(TWIST_PUB, 10);

    // Publish rate
    ros::Rate loop_rate(10);

    // The twist message to publish
    geometry_msgs::Twist msg;
    msg.linear.x = 0.7f;
    msg.angular.z = 0.0f;

    // Publish loop
    while(ros::ok())
    {
        // Publish the message
        pub.publish(msg);

        // Wait for ROS (time based on loop_rate above)
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("L2Bot go_fwd publishing!");
    ros::spin();

    return 0;
}
