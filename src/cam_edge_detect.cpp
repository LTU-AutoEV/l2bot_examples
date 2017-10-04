#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// Change this value if you are subscribing to a different camera
#define CAM_TOPIC "/camera_1/cam_pub/image_raw"

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
    void getEdges(cv::Mat& src, cv::Mat& dst);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
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
    cv::Mat edges;
    edges.create(cv_ptr->image.size(), cv_ptr->image.type());
    getEdges(cv_ptr->image,  edges);

    // Show preview window
    cv::imshow(CVWIN_PREVIEW, edges);

    // Update GUI Window
    cv::waitKey(3);

}


/**
 * Edge Detection Example
 * ======================
 *
 * An example function which detects the edges of the input image.
 * The image is first converted to grayscale. Then, amedian blur
 *   is applied to increase performance of the Canny edge detect
 *   function. Finally, the edged are dialated using MORPH_RECT.
 */
void SimpleCamSub::getEdges(cv::Mat& src, cv::Mat& dst)
{
    // Convert the source to grayscale
    cv::Mat src_gray;
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);

    // Edge detection parameters
    int lowThreshold = 65;
    int rat = 3;
    int kernel = 1;
    int blur = 1;
    int dilation_type = cv::MORPH_RECT;
    int dilation_size = 5;

    // These variables must be odd
    int kernel_size = kernel*2+1;
    int blur_size = blur*2+1;

    // Apply median blur and edge detect
    cv::medianBlur( src_gray, dst, blur_size );
    cv::Canny( dst, dst, lowThreshold, lowThreshold*rat, kernel_size );

    // DIaliate edges
    cv::Mat dilate_element = cv::getStructuringElement( dilation_type,
            cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
            cv::Point( dilation_size, dilation_size ) );
    cv::dilate( dst, dst, dilate_element );
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
