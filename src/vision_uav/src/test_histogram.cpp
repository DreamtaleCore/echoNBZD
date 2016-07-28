// opencv lib
#include <opencv2/opencv.hpp>

// ros
#include <ros/ros.h>

// cv_bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//msg
#include "geometry_msgs/Point.h"

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_histogram_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;


    Mat sample = imread("/home/odroid/");

























    imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
    pub = nh.advertise<ukftest::markerInfo>("marker_pose",200);
    ros::spin();
    return 0;
}
