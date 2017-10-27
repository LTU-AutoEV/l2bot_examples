#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <math.h>

#define JOY_SUB "/joy"

/* Class JoySetup
 * Convert gamepad input to a geometry_msgs::Twist
 *
 * Publish:     Twist on TWIST_PUB
 * Subscribe:   Joy on "/joy"
 */
class JoySetup
{
public:
    JoySetup();

private:

    // Callback function
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    // ROS node handle
    ros::NodeHandle nh_;

    // Publisher and subscriber
    ros::Subscriber joy_sub_;
};


// Constructor
// Set up publisher and subscriber
JoySetup::JoySetup() : nh_{"~"}
{
    // Subscribe to the controller
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(JOY_SUB, 10, &JoySetup::joyCallback, this);
}


// Callback function
// Take input from contoller and create a vector from the input
void JoySetup::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    int n = sizeof(joy->axes) / sizeof(joy->axes[0]);
    int max_idx = -1;
    float max_val = 0;
    for (int i = 0; i < n; i++) {
        if (fabs(joy->axes[i]) > max_val) {
            max_val = fabs(joy->axes[i]);
            max_idx = i;
        }
    }

    ROS_INFO_STREAM("Max idx: " << max_idx << " with value " << max_val);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_setup");
    JoySetup joy_setup;

    ros::spin();
}
