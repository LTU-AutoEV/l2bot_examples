#include <ros/ros.h>
#include <std_msgs/String.h>

class HelloWorldSub
{
public:
    HelloWorldSub();
    void helloWorldCB(const std_msgs::String& msg);

private:
    // Node handle and subscriber are private class member
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

HelloWorldSub::HelloWorldSub()
    :nh_{"~"}
{
    sub_ = nh_.subscribe("/hello_world", 10, &HelloWorldSub::helloWorldCB, this);
}

void HelloWorldSub::helloWorldCB(const std_msgs::String& msg)
{
    ROS_INFO_STREAM(msg.data);
}

int main(int argc, char** argv)
{

    // Init ros
    ros::init(argc, argv, "hello_world_sub");

    // All we have to do is create an instance of the object
    HelloWorldSub hw{};

    ros::spin();
    return 0;
}
