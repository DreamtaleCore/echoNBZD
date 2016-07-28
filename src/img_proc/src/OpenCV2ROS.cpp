/// OpenCV2ROS.cpp
/// Show a node that transform a img to cv::Mat
/// and draw a circle on the img, republish it to OpenCV
/// @yunfei Learning in OpenCV 2015/6/5

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WIN[] = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
      : it_(nh_)
    {
        image_pub_ = it_.advertise("out", 1);
        image_sub_ = it_.subscribe("in", 1,
            &ImageConverter::imageCb, this);

        cv::namedWindow(WIN);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(WIN);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg,enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return ;
        }
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50,50), 10,
                        CV_RGB(255,0,0));

        cv::imshow(WIN, cv_ptr->image);
        cv::waitKey(3);

        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
