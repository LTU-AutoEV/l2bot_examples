// ROS and messages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// L2Bot Controller Topic
#define TWIST_PUB "/rb_drive/rb_drive/twist_cmd"
// Time to drive forward (in seconds)
#define FWD_TIME 8.0

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
    msg.linear.x = 2.0f;
    msg.angular.z = 0.0f;

    ros::Time begin = ros::Time::now();

    ROS_INFO_STREAM("l2bot_example 'forward' publishing for " << FWD_TIME << " seconds!");

    // Publish loop
    while(ros::ok())
    {
        // Publish the message
        pub.publish(msg);

        // Wait for ROS (time based on loop_rate above)
        ros::spinOnce();
        loop_rate.sleep();

        // Get runtime duration
        ros::Time end = ros::Time::now();
        ros::Duration dur = end - begin;

        // 5 seconds
        if (dur.toSec() > FWD_TIME) {
            // Publish stop command
            msg.linear.x = 0.0f;
            pub.publish(msg);

            // Exit loop
            break;
        }
    }

    ROS_INFO_STREAM("l2bot: forward complete!");
    ros::spin();

    return 0;
}
