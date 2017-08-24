#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <math.h>

#define JOY_SUB "/joy"
#define STATE_PUB "twist_cmd"
#define TWIST_PUB "/turn_flag/flag"

/* Class JoyNav
 * Convert gamepad input to a geometry_msgs::Twist
 *
 * Publish: 	Twist on "~/twist_cmd"
 *				UInt32 on "/turn_flag/flag"
 * Subscribe: 	Joy on "/joy"
 */
class JoyNav
{
public:
	JoyNav();

	static constexpr int LINEAR_AXIS = 2;
	static constexpr int ANGULAR_AXIS = 3;
	static constexpr int BUTTON_LB = 4;
	static constexpr int BUTTON_RB = 5;
	static constexpr int BUTTON_Y = 3;
	static constexpr int BUTTON_X = 2;
	static constexpr int BUTTON_B = 1;
	static constexpr int BUTTON_A = 0;
	static constexpr float MULTIPLIER = 3.0f;

	// Must match rbdrive
	static constexpr int NAV_FLAG = 0;
	static constexpr int LEFT_FLAG = 1;
	static constexpr int RIGHT_FLAG = 2;
	static constexpr int LANE_CHANGE_FLAG = 3;
	static constexpr int STOP_FLAG = 4;
	static constexpr int SWAP_LANE_STATE_FLAG = 10;

private:

	// Callback function
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	// ROS node handle
	ros::NodeHandle nh_;

	// Publisher and subscriber
	ros::Publisher twist_pub_;
	ros::Publisher state_pub_;
	ros::Subscriber joy_sub_;
};


// Constructor
// Set up published and subscriber
JoyNav::JoyNav() : nh_{"~"}
{
	// Subscribe to the controller
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(JOY_SUB, 10, &JoyNav::joyCallback, this);

	// Publish control vectors
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>(TWIST_PUB, 100);
	state_pub_ = nh_.advertise<std_msgs::UInt32>(STATE_PUB, 100);
}


// Callback function
// Take input from contoller and create a vector from the input
void JoyNav::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	// Get right-left and fwd-bkwd values
	// Values range from -1 to 1
	float rl = joy->axes[ANGULAR_AXIS];
	float fb = (1 - (joy->axes[LINEAR_AXIS]))/2.0f;

	// Create a vector
	geometry_msgs::Twist vec;
	vec.linear.x = MULTIPLIER * fb;
	vec.angular.z = atan((MULTIPLIER / 10) * rl);

	// Publish the vector
	twist_pub_.publish(vec);


	// Flags
	if (joy->buttons[BUTTON_X] == 1) {
		std_msgs::UInt32 s;
		s.data = LEFT_FLAG;
		state_pub_.publish(s);
	} else if (joy->buttons[BUTTON_B] == 1) {
		std_msgs::UInt32 s;
		s.data = RIGHT_FLAG;
		state_pub_.publish(s);
	} else if (joy->buttons[BUTTON_Y] == 1) {
		std_msgs::UInt32 s;
		s.data = STOP_FLAG;
		state_pub_.publish(s);
	} else if (joy->buttons[BUTTON_LB] == 1) {
		std_msgs::UInt32 s;
		s.data = SWAP_LANE_STATE_FLAG;
		state_pub_.publish(s);
	} else if (joy->buttons[BUTTON_RB] == 1) {
		std_msgs::UInt32 s;
		s.data = LANE_CHANGE_FLAG;
		state_pub_.publish(s);
	} else if (joy->buttons[BUTTON_A] == 1) {
		std_msgs::UInt32 s;
		s.data = NAV_FLAG;
		state_pub_.publish(s);
	}
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_nav");
	JoyNav joy_nav;

	ros::spin();
}
