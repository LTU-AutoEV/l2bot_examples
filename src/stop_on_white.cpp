#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// Change this value if you are subscribing to a different camera
#define CAM_TOPIC "/camera_1/cam_pub/image_raw"
#define TWIST_PUB "/rb_drive/rb_drive/twist_cmd"

#define CVWIN_PREVIEW "threshold preview"

/**
 * Stop on White
 * =======================
 *
 * In this example we use a class to modularize the functionality
 *   of this node. We can include several member functions and
 *   variables which hide the functionality from main().
 */
class StopOnWhite
{
public:
    StopOnWhite();
    ~StopOnWhite();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

private:
    float countWhite(const cv::Mat& src, cv::Mat& dst);

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
 *   needs to instantiate the StopOnWhite object once and do nothing
 *   else (see main() below).
 *
 * In this case, we only need to set up the image subscriber
 */
StopOnWhite::StopOnWhite()
    :nh_{"~"}, it_{nh_}
{
    // Subscribe to the camera publisher node
    image_sub_ = it_.subscribe(CAM_TOPIC, 1, &StopOnWhite::imageCb, this);

    // Publish on the l2bot twist command topic
    pub_ = nh_.advertise<geometry_msgs::Twist>(TWIST_PUB, 10);
}



/**
 * Destructor
 * ==========
 *
 * Destroy CV windows
 */
StopOnWhite::~StopOnWhite()
{
    cv::destroyWindow(CVWIN_PREVIEW);
}



/**
 * Callback function
 * =================
 *
 * Called once every time a image is published on the topic this
 *   node is subscribed to. The image is passed to the function as
 *   a ImageConstPtr
 */
void StopOnWhite::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    // Apply a threshold and count the number of white pixels
    cv::Mat preview;
    float white_visible = countWhite(cv_ptr->image, preview);

    // Uncomment to print pixel ratio to terminal
    ROS_INFO_STREAM("white/all pixel ratio: " << white_visible);

    // If the number of white pixels is above a certain percent, stop
    geometry_msgs::Twist twist;
    if (white_visible > 0.50)
    {
        twist.linear.x = 0.0;
    }
    else
    {
        twist.linear.x = 2.0;
    }
    pub_.publish(twist);

    // Show preview window
    cv::imshow(CVWIN_PREVIEW, preview);

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
float StopOnWhite::countWhite(const cv::Mat& src, cv::Mat& preview)
{
    // Convert the source to grayscale
    cv::Mat src_gray;
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);

    // Blur the image to reduce noise
    cv::medianBlur(src_gray, src_gray, 11);

    // Threshold parameters
    int const threshold_type = cv::THRESH_OTSU;
    int const threshold_value = 3;
    int const max_value = 255;
    int const max_type = 4;
    int const max_BINARY_value = 255;

    cv::threshold(src_gray, preview, threshold_value, max_BINARY_value, threshold_type);

    // The number of white pixels
    int white_count = cv::countNonZero(preview);

    // Function return ratio #white / #total
    return (float)white_count / (float)(src.rows * src.cols);
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop_on_white");

    // Create a StopOnWhite object.
    // Since initilization code is in the constructor, we do
    //   not need to do anythong else with this object
    StopOnWhite sd{};

    ROS_INFO_STREAM("stop_on_white running!");
    ros::spin();
    return 0;
}
