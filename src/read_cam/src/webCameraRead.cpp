/* @Camera processing nodes
 * November 14 edition, has joined the：
 * Image acquisition and transfer
 * Removing distortions
 * Histogram strengthen
 * Distortion has been removed
 *
 * @Note: * .xml file to modify the calibration of the location and name
 * @Author: DreamTale
 * @Date: 2014-11-14
 */

#define CAMERA_NUM 1
//#define CAMERA_NUM "/home/mylove/Pictures/aruco/single/video.avi"

//#define CAMERA_NUM "/home/mylove/ws/src/movSRC/tilted_face.avi"

//include system in/out libraries
#include<sstream>
#include<iostream>
#include<stdlib.h>
//#include <string>

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

/* this file has the coordinate with opencv photo:
 *  (0,0)+------------------------>u
 *       |
 *       |
 *       |
 *       \/v
 *  the number of distence between the circle and UAV
 *   is depend on the hight of uav
 */

using namespace cv;
using namespace std;

//String CalibrationCascadeName = "/home/odroid/ws/src/read_cam/camera.xml";
int i = 0;
double scale = 1.4;//Scaling detected

//Mat cameraMatrix, distCoeffs;
//Mat map1, map2;
Mat view, rview;
Mat Image, ImageGRAY, mid;

int imgWidth, imgHeight;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_camera_reader");
    ros::NodeHandle n,nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("uav_cam/image", 3);

//    string cameraCalibrationInfo = CalibrationCascadeName ;
//    FileStorage fs(cameraCalibrationInfo, FileStorage::READ);
//    fs["camera_matrix"] >> cameraMatrix;
//    fs["distortion_coefficients"] >> distCoeffs;
//    fs["image_width"] >> imgWidth;
//    fs["image_height"] >> imgHeight;
//    fs.release();

//    std::cout << cameraMatrix << std::endl;
//    std::cout << distCoeffs << std::endl;
//    std::cout << imgWidth << std::endl;
//    std::cout << imgHeight << std::endl;

    cv_bridge::CvImagePtr cvPtr;
    cv_bridge::CvImage outMsg;

    ros::Rate loopRate(30);

    geometry_msgs::Point32 msg;
    VideoCapture cap(CAMERA_NUM);
    cap.set(CV_CAP_PROP_FRAME_WIDTH ,640);///////////////////////change here!!here!!here!!!!
    cap.set(CV_CAP_PROP_FRAME_HEIGHT ,480);///////////////////////change here!!here!!here!!!!
    cap.set(CV_CAP_PROP_EXPOSURE ,60);

    Size imageSize;
    int count=0;

    if( n.ok() )
    {
        cap>>view;
        imageSize = view.size();

        //distoration map
//       initUndistortRectifyMap(cameraMatrix,distCoeffs,
//                               cv::noArray(),
//                               cameraMatrix,imageSize,
//                               CV_16SC2,
//                               map1,map2);
        //std::cout << map1 << std::endl;
        //std::cout << map2 << std::endl;
//       printf("test image get and remap down");

        while (nh.ok())
        {
            cap>>view;
            //imshow("Source View", view);
            Image = view;

            //undistort(Image, mid, cameraMatrix, distCoeffs);
//           remap(view, Image, map1, map2, INTER_LINEAR);   //remap the image,over
            imshow("Undistored View", Image);
            imwrite("Image.jpg", Image);
            //resize(Image, Image, Size(320,240));
       //     std::cout<<"i've got a image!!!"<<endl;
       //     std::cout<<"Image Info: "<<"height-"<<Image.rows<<" width-"<<Image.cols<<std::endl;
            //outMsg.image = grayImg;
            outMsg.image = Image;
            outMsg.header.stamp = ros::Time::now();
            outMsg.encoding = "bgr8";

            pub.publish(outMsg.toImageMsg());
            ros::spinOnce();
            loopRate.sleep();

            waitKey(1);
            char c = cvWaitKey(1);
            if((char)c == 'c')
            { break; }
        }
    }

    return 0;
}
