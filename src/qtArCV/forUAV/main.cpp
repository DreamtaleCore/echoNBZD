#include "opencvHeaders.h"
#include "stdHeaders.h"
#include "arucoHeaders.h"
#include <ros/advertise_options.h>
#include <ros/message.h>

#include <tf/tf.h>

using namespace ros;

int main(int argc, char* argv[])
{
    Mat src;

    CameraParameters camParam;
    MarkerDetector mDetector;
    vector<Marker> markers;
    src = imread("/home/mylove/Pictures/aruco/4.jpg");
    camParam.readFromXMLFile("/home/mylove/ws/src/vision_uav/ca65.yml");
    float markersize = atof("0.25");

    mDetector.detect(src, markers, camParam, markersize);

    for (unsigned int i=0;i< markers.size();i++)
    {
        cout<< "ID :" <<endl<< markers[i].id   <<endl;
        cout<< "Txyz :" <<endl<< markers[i].Tvec <<endl;
        cout<< "Rxyz :" <<endl<< markers[i].Rvec <<endl;
        cout<< "ssize :" <<endl<< markers[i].ssize <<endl;
        cout<< "point1 | "<< markers[i][0].x<<" | "<< markers[i][0].y<<endl;
        cout<< "point2 | "<< markers[i][1].x<<" | "<< markers[i][1].y<<endl;
        cout<< "point3 | "<< markers[i][2].x<<" | "<< markers[i][2].y<<endl;
        cout<< "point4 | "<< markers[i][3].x<<" | "<< markers[i][3].y<<endl<<endl;
        //cout<<markers[i]<<endl<<endl;

        markers[i].draw(src,Scalar(0,0,255),2);

        CvDrawingUtils::draw3dCube(src, markers[i], camParam);

    }
    cv::imshow("in",src);
    cv::waitKey(0);//wait for key to be pressed
    cout<< "DETECT done."<<endl;

    return 0;
}
