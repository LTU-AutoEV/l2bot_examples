#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// Change this value if you are subscribing to a different camera
#define CAM_TOPIC "/camera_1/cam_pub/image_raw"
#define TWIST_PUB "/rb_drive/rb_drive/twist_cmd"

// The name of the preview window
#define CVWIN_PREVIEW "cam_edge_detect preview"


/**
 * Simple Camera Subscriber
 * =======================
 *
 * In this example we use a class to modularize the functionality
 *   of this node. We can include several member functions and
 *   variables which hide the functionality from main().
 */
class SimpleCamSub
{
public:
    SimpleCamSub();
    ~SimpleCamSub();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

private:
    void countWhite(cv::Mat& src, cv::Mat& dst);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_;
};


/**
 * Constructor
 * ===========
 *
 * Do all initilization code here. This way, our main() function only
 *   needs to instantiate the SimpleCamSub object once and do nothing
 *   else (see main() below).
 *
 * In this case, we only need to set up the image subscriber
 */
SimpleCamSub::SimpleCamSub()
    :nh_{"~"}, it_{nh_}
{
    image_sub_ = it_.subscribe(CAM_TOPIC, 1, &SimpleCamSub::imageCb, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>(TWIST_PUB, 10);
}



/**
 * Destructor
 * ==========
 *
 * Destroy CV windows
 */
SimpleCamSub::~SimpleCamSub()
{
    cv::destroyWindow(CVWIN_PREVIEW);
}



/**
 * Callback function
 * =================
 *
 * Called once every time a image is published on the topic this
 *   node is subscribed to. The image is passed to the function as
 *   a ImageConstPtr. A few lines of code validate the image and
 *   convert it into a cv::Mat
 */
void SimpleCamSub::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //Convert to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Run edge detection function
    cv Mat::preview;
    float white_visible = countWhite(cv_ptr->image);

    // Uncomment to print pixel ratio to terminal
    ROS_INFO_STREAM("white/all pixel ratio: " << white_visible);

    // If the number of white pixels is above a certain percent, stop
    geometry_msgs::Twist msg;
    if (white_visible > 0.50) 
    {
        msg.linear.x = 0.0;
    }
    else
    {
        msg.linear.x = 2.0;
    }
    pub_.publish(msg);

    // Show preview window
    cv::imshow("Threshold Preview", preview);

    // Update GUI Window
    cv::waitKey(3);

}


/**
 * Count White Pixels Example
 * ==========================
 * This function uses dynamic thresholding to binarize the imput image.
 * It then counts the number of white pixels and returns (#white)/(#total)
 *
 * https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html?highlight=threshold
 */
float SimpleCamSub::countWhite(const cv::Mat& src, cv::Mat& preview)
{
    // Convert the source to grayscale
    cv::Mat src_gray;
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);

    // Binary threshold
    double max_value = 255.0;
    int block_size = 3;

    cv::adaptiveThreshold(src_gray, preview,
            max_value,
            ADAPTIVE_THRESH_GAUSSIAN_C
            CV_THRESH_BINARY,
            block_size);

    int white_count = cv::countNonZero(preview);

    return (float)white_count / (float)(src.rows * src.cols)
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_edge_detect");

    // Create a SimpleCamSub object.
    // Since initilization code is in the constructor, we do
    //   not need to do anythong else with this object
    SimpleCamSub sd{};

    ROS_INFO_STREAM("cam_edge_detect running!");
    ros::spin();
    return 0;
}
