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
Mat view, mono_view, rgb_view;
ros::Publisher data_pub;

/**
* @function stepGetBackground
* @brief Use addWeighted function and divided method
*			to get a better background
*/
void stepGetBackground(Mat _src[], Mat &out, int sum)
{
    int i, j, tempSum = sum;
    Mat * src = new Mat[sum];
    for (i = 0; i < sum; i++)
        src[i] = _src[i];

    while (tempSum > 0)
    {
        cout <<"1265464654"<<endl;

        for (int j = 0; j < tempSum; j = j + 2)
        {
            addWeighted(src[j], 0.5, src[j + 1], 0.5, 0.0, src[j / 2]);
        }
        tempSum = tempSum / 2;
        cout <<"12122212121"<<endl;

    }
    out = src[0];
    src->release();
}

/**
 * @brief imageCallback
 * @param msg
 */
int frameNum = 0;
Mat src[findNum];
Mat backgroundImg;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//    cout<< "get a Image."<<endl;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    Image = cv_ptr->image;
    if(frameNum < findNum)          // generate the background image
    {
        src[frameNum] = Image;
        frameNum ++;
    }

    stepGetBackground(src, backgroundImg, findNum);

   //resize(Image,Image,Size(320,240));
    if(Image.empty())
        return;
    mono_view = Image;
//    imshow("gary img",mono_view);
    cvtColor(Image, Image, CV_GRAY2RGB);
//    imshow("movingImage", Image);
    if(frameNum >= findNum)
    imshow("background", backgroundImg);
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

    src->release();
    backgroundImg.release();

    ros::spin();
    return 0;
}
