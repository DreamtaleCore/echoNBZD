//include system in/out libraries
#include "../../include/stdHeader.h"

//include ros libraries
#include<ros/ros.h>

//include messege libraries
#include "../../include/msgsHeader.h"

//include opencv libraries
#include "../../include/opencvHeader.h"

//include ros transport&bridge libraries
#include "../../include/transHeader.h"

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
//cv_bridge::CvPointPtr outMsg;
Mat Image;
Mat view, mono_view, rgb_view;
String face_cascade_name = "/home/mylove/ws/haarcascade_frontalface_alt2.xml";
CascadeClassifier face_cascade;
ros::Publisher data_pub;

void detectAndDisplay(Mat frame);

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc,char **argv)
{
    ros::init(argc, argv, "test_color_node");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;
    data_pub = nh.advertise<std_msgs::String>("movDetec", 1000);

    if (!face_cascade.load(face_cascade_name)){ printf("--(!)Error loading face cascade\n"); return -1; };

    imgSub = it.subscribe("/uav_cam/image", 5, imageCallback);

    ros::spin();
    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//    cout<< "get a Image."<<endl;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    Image = cv_ptr->image;

   //resize(Image,Image,Size(320,240));
    if(Image.empty())
	return;
    mono_view = Image;
//    imshow("gary img",mono_view);
    cvtColor(Image, Image, CV_GRAY2RGB);
//    rgb_view = Image;
//    imshow("color img",rgb_view);
    detectAndDisplay(Image);
    waitKey(1);
//    cout<< "ALL done."<<endl;

}

/** @function detectAndDisplay */
void detectAndDisplay(Mat frame)
{
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    //-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

    for (size_t i = 0; i < faces.size(); i++)
    {
        rectangle(frame, faces[i], Scalar(0, 255, 255));
        cout << "FrameX = " << faces[i].x << "  FrameY = " << faces[i].y << endl;
        cout << "FrameWidth = " << faces[i].width << "  FrameHeight = " << faces[i].height << endl << endl;

        // Store data and publish it
        std_msgs::String msg;
        //std_msgs::Int16 send_data[4];
        //send_data[0].data = faces[i].x;  send_data[1].data = faces[i].y;
        //send_data[2].data = faces[i].width;  send_data[3].data = faces[i].height;
        stringstream ss;
        ss << faces[i].x <<","<< faces[i].y <<","
               << faces[i].width <<","<< faces[i].height <<",";
        msg.data = ss.str();
        //msg = send_data;
        data_pub.publish(msg);
        //data_pub.publish(send_data);
    }
    //-- Show what you got
    imshow("face detect", frame);
}
