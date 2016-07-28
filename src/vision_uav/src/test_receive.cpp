//include system in/out libraries
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>


//include ros libraries
#include<ros/ros.h>

//include messege libraries
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"

//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/legacy/compat.hpp>
#include "opencv2/ml/ml.hpp"
#include "opencv2/objdetect/objdetect.hpp"

//include ros transport&bridge libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

#define THRESHOLD 80
#define SCALE 2
/* ****CONST DEFINITION**** */
//
//      * ------------------->  x
//      |
//      |
//      |
//     \/ y 
// below are the positions of markers relative to home point

Mat view, mono_view, rgb_view;

ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);


int main(int argc,char **argv)
{
    ros::init(argc, argv, "test_receive_node");
	ros::NodeHandle nh;
    cout<< "here is the 1st."<<endl;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imgSub;

    imgSub = it.subscribe("/uav_cam/image", 5, imageCallback);

    ros::spin();
	return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cout<< "get a Image."<<endl;
    Mat Image;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Image = cv_ptr->image;
    imshow("debug",Image);

    namedWindow("gary img");
    //resize(Image,Image,Size(320,240));

    rgb_view = Image;
    imshow("color img",rgb_view);
    imwrite("color img1.jpg",rgb_view);

    cvtColor(Image, Image, CV_RGB2GRAY);

    mono_view = Image;
    imshow("gary img",mono_view);
    imwrite("gary img1.jpg",mono_view);

    cout<< "ALL SAVE done."<<endl;
    waitKey(1);
}
