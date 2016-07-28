#include <ros/ros.h>
#include <iostream>

#include "aruco_lib/aruco.h"
#include "aruco_lib/cvdrawingutils.h"
#include "aruco_lib/marker.h"

#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace aruco;
using namespace ros;

int main(int argc,char **argv)
{
        ros::init(argc, argv, "aruco_test_on_ros1");
        ros::NodeHandle nh;

        CameraParameters camParam;
        MarkerDetector MDetector;
        vector<Marker> Markers;
        //read the input image
        cv::Mat InImage;
        InImage=cv::imread(argv[1]);

        //read camera parameters
        camParam.readFromXMLFile("/home/mylove/ws/src/optObjUAV/ca65.yml");
        float markerSize = atof("0.25");

        //Ok, let's detect
        MDetector.detect(InImage, Markers, camParam, markerSize);

        //for each marker, draw info and its boundaries in the image
        for (unsigned int i=0;i<Markers.size();i++)
        {
            cout<< "ID :" <<endl<< Markers[i].id   <<endl;
            cout<< "Txyz :" <<endl<< Markers[i].Tvec <<endl;
            cout<< "Rxyz :" <<endl<< Markers[i].Rvec <<endl;
            cout<< "ssize :" <<endl<< Markers[i].ssize <<endl;
            cout<< "point1 | "<< Markers[i][0].x<<" | "<< Markers[i][0].y<<endl;
            cout<< "point2 | "<< Markers[i][1].x<<" | "<< Markers[i][1].y<<endl;
            cout<< "point3 | "<< Markers[i][2].x<<" | "<< Markers[i][2].y<<endl;
            cout<< "point4 | "<< Markers[i][3].x<<" | "<< Markers[i][3].y<<endl<<endl;
            //cout<<Markers[i]<<endl<<endl;

            Markers[i].draw(InImage,Scalar(0,0,255),2);

            CvDrawingUtils::draw3dCube(InImage, Markers[i], camParam);

        }
        cv::imshow("in",InImage);
        cv::waitKey(0);//wait for key to be pressed
        cout<< "DETECT done."<<endl;


        ros::spin();
        return 0;
}
