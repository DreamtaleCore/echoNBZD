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
Mat InImage;
geometry_msgs::Vector3 OutMsg;



Mat Image, ImageGRAY, mid,
    imageEqualize;
int center_x,center_y;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

ros::Publisher pub;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "test_hough");
    ros::NodeHandle nh;
    cout<< "here is the 1st."<<endl;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;

    imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
    pub = nh.advertise<geometry_msgs::Vector3>("img_pos",20);

    ros::spin();
    return 0;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cout<< "something wrong?"<<endl;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    Image = cv_ptr->image;
    Image.copyTo(mid);

    resize(Image, Image,Size(480,360));
    //cvtColor(Image, Image, CV_GRAY2RGB);

    GaussianBlur(Image, Image, Size(9,9),2,2);

    imshow("Image",Image);

    waitKey(1);

    double t = 0;
    t = (double)cvGetTickCount();//Used to calculate the execution time of the algorithm

    vector<Vec3f> circles;

    HoughCircles(Image, circles, CV_HOUGH_GRADIENT, 1.5 , 10, 200, 100, 0, 0);		//»ô·ò±ä»» Ô²

    center_x = 0;
    center_y = 0;
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        center_x = center.x - 240;
        center_y = center.y - 180;

        circle(Image, center, 5, Scalar(0, 255, 0), -1, 8, 0);
        circle(Image, center, radius, Scalar(255, 0, 0), 3, 8, 0);

        printf("the circle %d \n", i);
        printf("the Radius	=	%d\n", radius);
        printf("the Center	:	x=	%d ;y=	%d\n\n", center_x, center_y);
    }

    imshow("debug", Image);

    std::cout<<"Frame Info: "<<"height-"<<Image.rows<<" width-"<<Image.cols<<std::endl;

    t = (double)cvGetTickCount() - t;//Subtraction algorithm execution time for
    printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );


    OutMsg.x = center_x/1.5;
    OutMsg.y = center_y/1.5;
    OutMsg.z = 0;

FUNC_END:
    pub.publish( OutMsg );
}

