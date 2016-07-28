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
const int findNum = 256;
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
ros::Publisher data_pub;

void findCircle(Mat src)
{
    int kvalue = 15;
    //    imshow("Src", src_color);
    //    imshow("Gray", src_gray);

    Mat bf;
    bf = src;
    //    bilateralFilter(src, bf, kvalue, kvalue * 2, kvalue / 2);
    GaussianBlur(bf, bf, Size(9, 9), 2, 2);
        imshow("Filter", bf);

    Mat dst = src;

    vector<Vec3f> circles;
    HoughCircles(bf, circles, CV_HOUGH_GRADIENT, 1, bf.rows/8, 200, 100, 0, 0);

    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        circle(dst, center, 3, Scalar(0, 255, 255), -1, 8, 0);
        circle(dst, center, radius, Scalar(255, 0, 0), 3, 8, 0);

        cout << cvRound(circles[i][0]) << "\t"
                                       << cvRound(circles[i][1]) << "\t"
                                                                 << cvRound(circles[i][2]) << endl << endl;
    }

    imshow("Detect Flow", dst);
}

/**
 * @brief imageCallback
 * @param msg
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //    cout<< "get a Image."<<endl;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    Image = cv_ptr->image;
    //    Image = imread("/home/mylove/round1.jpg");
    //resize(Image,Image,Size(320,240));
    //cvtColor(Image, Image, CV_GRAY2RGB);
    //    cvtColor(Image, Image, CV_BGR2GRAY);

    if(Image.empty())
        return;
    else
    {
        findCircle(Image);
    }
    waitKey(1);
    //    cout<< "ALL done."<<endl;

}

/**
 * @brief main
 * @param argc, argv
 * @return
 */
int main(int argc,char **argv)
{
    ros::init(argc, argv, "test_color_node");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;
    data_pub = nh.advertise<std_msgs::String>("datectMoving", 1000);

    imgSub = it.subscribe("/uav_cam/image", 5, imageCallback);

    ros::spin();
    return 0;
}
