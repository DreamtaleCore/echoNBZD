#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include <sstream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_reader");
    ros::NodeHandle nh,n;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("uav_cam/image", 3);
    ros::Publisher cir_pub = n.advertise<std_msgs::String>("cir_data", 500);
    cv_bridge::CvImagePtr cvPtr;
    cv_bridge::CvImage outMsg;
    cv::Mat colorImg, grayImg;
    ros::Rate loopRate(10);
    CvCapture* capture;
    std_msgs::String msg;
    std::stringstream st;
    // Read the video stream
    capture = cvCaptureFromCAM( -1);
    //cv::VideoCapture cap(0);
    int count=0,r=0,x=0,y=0;
    namedWindow("Capture_Video",CV_WINDOW_AUTOSIZE);
    if( capture )
    {
        while( true )
        {
            colorImg = cvQueryFrame( capture );
            //cap >> colorImg;
            //cv::cvtColor(colorImg, grayImg, CV_RGB2GRAY);
            grayImg=colorImg;
            Mat mid,imageGreenChannel,pFrame;
            vector<Mat>channels;

            pFrame = colorImg;
            Mat Image = pFrame;

            cvtColor(Image, mid, CV_BGR2GRAY);
            split(Image, channels);
            imageGreenChannel = channels.at(1);
            vector<Vec3f> circlesGreen;
            HoughCircles(mid, circlesGreen, CV_HOUGH_GRADIENT, 1.5, 5, 200, 150, 2, 0);		//»ô·ò±ä»» Ô²
            for (size_t i = 0; i < circlesGreen.size(); i++)
            {
                Point center(cvRound(circlesGreen[i][0]), cvRound(circlesGreen[i][1]));
                int radius = cvRound(circlesGreen[i][2]);

                circle(Image, center, 5, Scalar(0, 255, 0), -1, 8, 0);
                circle(Image, center, radius, Scalar(255, 0, 0), 3, 8, 0);

                //printf("the circle %d \n", i);
                printf("the Radius	=	%d\n", radius);
                printf("the Center	:	x=	%d ;y=	%d\n\n", center.x, center.y);

                r=radius;
                x=center.x;
                y=center.y;
            }
            imshow("imageGreenChannel", Image);


            st << "\ncir_data_output: "<<count++ <<"\n"<<"radius=:"<<r<<"\ncenter.x=:"<<x<<"\ncenter.y=:"<<y<<"\nTime=:"<<ros::Time::now();



            //std::cout<<"height: "<<grayImg.rows<<", width: "<<grayImg.cols<<std::endl;
            //outMsg.image = grayImg;
            outMsg.image = Image;
            outMsg.header.stamp = ros::Time::now();
            outMsg.encoding = "bgr8";

            msg.data = st.str();
            cir_pub.publish(msg);
            pub.publish(outMsg.toImageMsg());
            ros::spinOnce();
            loopRate.sleep();

            imshow("Capture_Video",colorImg);

            // Press 'c' to escape
            int c = waitKey(1);
            if( (char)c == 'c' ) { break; }
        }
    }
    return 0;

}

