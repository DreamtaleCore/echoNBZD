//include system in/out libraries
#include <iostream>

//include ros libraries
#include<ros/ros.h>

//include messege libraries
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Vector3.h"

//include opencv libraries
#include <opencv2/opencv.hpp>

//include ros transport&bridge libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//include ros aruco libraries
#include "aruco_lib/aruco.h"
#include "aruco_lib/cvdrawingutils.h"
#include "aruco_lib/marker.h"

using namespace cv;
using namespace std;
using namespace aruco;

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

ros::Publisher pub;

double MKsize;
Mat Rmtx;
float posX, posY, posZ;
float numOfMarkersDetected;
float center_x,center_y;
Mat pos_tmp;

CameraParameters camParam;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc,char **argv)
{
    ros::init(argc, argv, "aruco_test_on_ros2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    //read camera parameters
    camParam.readFromXMLFile("/home/mylove/ws/src/vision_uav/ca150.yml");

    cout<< "here is the 1st."<<endl;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;

    nh.param("MKsize", MKsize, double(27));

    imgSub = it.subscribe("/uav_cam/image", 5, imageCallback);
    pub = nh.advertise<geometry_msgs::Vector3>("img_pos",200);

    ros::spin();
    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cout<< "get a Image."<<endl;
    Mat Image;
    geometry_msgs::Vector3 outMsg;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Image = cv_ptr -> image;



    MarkerDetector MDetector;
    vector<Marker> Markers;

    //read the input image
    cv::Mat InImage;
    InImage = Image;

    //Ok, let's detect
    MDetector.detect(InImage, Markers, camParam, MKsize);

    //for each marker, draw info and its boundaries in the image
    for (unsigned int i=0;i<Markers.size();i++)
    {

        //cout<<Markers[i]<<endl;
        //cout<<Markers[i].Rvec<<endl;
        //cout<<Markers[i].id<<endl;
        cout<<Markers[i].Tvec<<endl;
        //cout<<Markers[i].ssize<<endl;
        //cout<<Markers[i][0].x<<" "<<Markers[i][0].y<<endl;
        //cout<<Markers[i][1].x<<" "<<Markers[i][1].y<<endl;
        //cout<<Markers[i][2].x<<" "<<Markers[i][2].y<<endl;
        //cout<<Markers[i][3].x<<" "<<Markers[i][3].y<<endl;

        Markers[i].draw(InImage,Scalar(0,255,255),2);
        CvDrawingUtils::draw3dCube(InImage, Markers[i], camParam);
    }

    numOfMarkersDetected = Markers.size();
    if (numOfMarkersDetected == 0)
    {
        outMsg.x = 0;
        outMsg.y = 0;
        outMsg.z = 0;
        pub.publish(outMsg);
        //return;
    }
    else
    {
        Rodrigues(Markers[0].Rvec, Rmtx);
        pos_tmp = -Rmtx.t()*Markers[0].Tvec;
        posX = pos_tmp.at<float>(0,0);
        posY = pos_tmp.at<float>(1,0);
        posZ = pos_tmp.at<float>(2,0);
        cout<<"posXYZ| "<<posX<<" | "<<posY<<" | "<<posZ<<endl;

        center_x = ( Markers[0][0].x + Markers[0][1].x + Markers[0][2].x + Markers[0][3].x )/4;
        center_y = ( Markers[0][0].y + Markers[0][1].y + Markers[0][2].y + Markers[0][3].y )/4;
        circle(InImage,Point2f(center_x,center_y),3,Scalar(0,255,255),4,8);

        outMsg.x = center_x - 320;
        outMsg.y = center_y - 240;
        outMsg.z = 1.0;
        pub.publish(outMsg);

        cout<<center_x<<" "<<center_y<<endl;

        //return;
    }


    cv::imshow("debug",InImage);

    //wait for key to be pressed
    cv::waitKey(1);
    cout<< "ALL done."<<endl<<endl;

    waitKey(1);
}
