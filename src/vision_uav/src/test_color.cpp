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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"


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

cv_bridge::CvImagePtr cv_ptr;
Mat Image;
Mat view, mono_view, rgb_view;

ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);


int main(int argc,char **argv)
{
    ros::init(argc, argv, "test_color_node");
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

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    Image = cv_ptr->image;

   //resize(Image,Image,Size(320,240));

    mono_view = Image;
    imshow("gary img",mono_view);

    cvtColor(Image, Image, CV_GRAY2RGB);
    rgb_view = Image;
    imshow("color img",rgb_view);
    waitKey(1);
    cout<< "ALL done."<<endl;

}
