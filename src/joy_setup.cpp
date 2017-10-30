#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <math.h>
#include <sstream>
#include <fstream>

#define JOY_SUB "/joy"
#define MAX_AXES 32

float axes[MAX_AXES];
int num_axes;
bool allow_feedback = false;
float axes_init[MAX_AXES];


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
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(JOY_SUB, 1, &JoySetup::joyCallback, this);

}


int getCurrentAxis() {
    int idx = -1;
    float max_diff = 0.0f;

    // Find max diff
    for (int i = 0; i < num_axes; i++) {
        float diff = fabs(axes_init[i] - axes[i]);
        if (diff > max_diff) {
            max_diff = diff;
            idx = i;
        }
    }

    if (max_diff == 0.0f) idx = -1;
    return idx;
}


// Callback function
// Take input from contoller and create a vector from the input
void JoySetup::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Initialize the number of axes
    if (num_axes == -1) {
        num_axes = sizeof(joy->axes) / sizeof(joy->axes[0]);
    }

    // Copy into axes array
    for (int i = 0; i < num_axes; i++)
        axes[i] = joy->axes[i];

    if (allow_feedback) {
        ROS_INFO_STREAM("Number of axes: " << num_axes);
        std::stringstream ss;
        ss << "[ ";
        for (int i = 0; i < num_axes; i++) {
            ss << axes[i] << " "; 
        }
        ss << "]";
        ROS_INFO_STREAM("Initial axes state: " << ss.str());

        ROS_INFO_STREAM("Current axes: " << getCurrentAxis());
    }
}

int getHeldAxisFromUser(std::string name)
{

    int ax = -1;
    int badcount = 0;
    ROS_INFO_STREAM("Hold " << name << " on your controller...");
    do {
        ros::Duration(4.0).sleep();
        ROS_INFO_STREAM("\tReading...");
        ros::spinOnce();
        ax = getCurrentAxis();

        if (ax == -1) {
            ROS_INFO_STREAM("No input read. Please hold the axes you would like to use for " << name << ".");
            badcount++;
            if (badcount > 1) {
                ROS_INFO_STREAM("Please make sure you are holding a joystick not a button.");
            }
        }
    } while (ax == -1);

    ROS_INFO_STREAM("Axis chosen: " << ax);
    return ax;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_setup");
    JoySetup joy_setup;

    // Init global variables
    num_axes = -1;
    for (int i = 0; i < MAX_AXES; i++)
        axes[i] = -100.0f;


    // First, prepare to read the initial state of the controller
    int counter = 0;
    ROS_INFO_STREAM("When ready, mash all buttons and joysticks on your controller!");
    do {
        if (num_axes != -1) {
            if (counter == 1) ROS_INFO_STREAM("Make sure to press ALL buttons and move ALL joysticks!");
            ROS_INFO_STREAM("\tReading...");
            counter++;
        }
        ros::Duration(2.0).sleep();
        ros::spinOnce();
    } while(num_axes == -1 || counter < 3);

    ROS_INFO_STREAM("Now RELEASE all buttons for a few seconds...");
    ros::Duration(4.0).sleep();
    ROS_INFO_STREAM("Now tap any button once.");
    ros::Duration(3.0).sleep();

    // Read the initial state
    ros::spinOnce();
    ROS_INFO_STREAM("\tReading...");
    ros::Duration(1.0).sleep();


    // Set up initial state
    for (int i = 0; i < num_axes; i++)
        axes_init[i] = axes[i];

    // Read axes
    int ax_fwd = getHeldAxisFromUser("FORWARD");
    int ax_left = getHeldAxisFromUser("RIGHT");

    // Write config file
    ROS_INFO_STREAM("Writing config file...");
    // The package path
    std::string package_path = ros::package::getPath("l2bot_examples");
    std::ofstream config;
    config.open(package_path + "/config/js_mappings.yaml");
    config << "axes_linear: " << ax_fwd << std::endl;
    config << "axes_angular: " << ax_left << std::endl;
    config.close();


    // Done!
    ROS_INFO_STREAM("Controller successfully set up!");
    ROS_INFO_STREAM("Press Ctrl-C to exit");

    ros::spin();
}
