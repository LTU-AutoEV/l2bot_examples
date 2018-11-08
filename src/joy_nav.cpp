#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <math.h>

#define JOY_SUB "/joy"
#define TWIST_PUB "/rb_drive/rb_drive/twist_cmd"
#define MULTIPLIER 3


/* Class JoyNav
 * Convert gamepad input to a geometry_msgs::Twist
 *
 * Publish:     Twist on TWIST_PUB
 * Subscribe:   Joy on "/joy"
 */
class JoyNav
{
public:
    JoyNav();

private:

    // Callback function
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    // ROS node handle
    ros::NodeHandle nh_;

    // Publisher and subscriber
    ros::Publisher twist_pub_;
    ros::Publisher state_pub_;
    ros::Subscriber joy_sub_;

    int axes_linear_;
    int axes_angular_;
};


// Constructor
// Set up publisher and subscriber
JoyNav::JoyNav() : nh_{"~"}
{
    // Subscribe to the controller
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(JOY_SUB, 10, &JoyNav::joyCallback, this);


    // Load JS Mappings
    if (!nh_.getParam("/joy_mappings/axes_linear", axes_linear_))
    {
        ROS_ERROR_STREAM("joy_nav: Could not load joystick configuration.");
        ROS_ERROR_STREAM("joy_nav: Please run `roslaunch l2bot_examples joy_setup.launch`");
        axes_linear_ = -1;
        axes_angular_ = -1;
    }
    nh_.getParam("/joy_mappings/axes_angular", axes_angular_);


    std::string twist_pub;
    // Load Topic
    if (!nh_.getParam("/topics/twist_cmd", twist_pub))
    {
        ROS_ERROR_STREAM("joy_nav: Could not load topic to publish on");
    }

    // Publish control vectors
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_pub, 100);

}


// Callback function
// Take input from contoller and create a vector from the input
void JoyNav::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Get right-left and fwd-bkwd values
    // Values range from -1 to 1
    float rl = joy->axes[axes_angular_];
    float fb = joy->axes[axes_linear_]; //(1 - (joy->axes[axes_linear_]))/2.0f;

    // Create a vector
    geometry_msgs::Twist vec;
    vec.linear.x = MULTIPLIER * fb * -1;
    vec.angular.z = atan(rl);

    // Publish the vector
    twist_pub_.publish(vec);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_nav");
    JoyNav joy_nav;

    ros::spin();
}
