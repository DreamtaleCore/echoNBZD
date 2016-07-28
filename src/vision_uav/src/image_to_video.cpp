#define FrameRate 10

#include "ros/ros.h"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

Mat Image;
int n;
IplImage* frame;
CvVideoWriter* video;
int FrameWidth = 640,FrameHeight = 480;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc,char** argv)
{
    ros::init(argc, argv, "get_video_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    cout<< "here is the start."<<endl;

    pnh.param("FrameWidth", FrameWidth, int(320));
    pnh.param("FrameHeight", FrameHeight, int(240));

    video = NULL;
    frame = NULL;

    video = cvCreateVideoWriter("/home/mylove/nomarl.avi", CV_FOURCC('X', 'V', 'I', 'D'), FrameRate, cvSize(FrameWidth,FrameHeight));
    //创建CvVideoWriter对象并分配空间,保存的文件名为camera.avi，编码要在运行程序时选择，大小就是摄像头视频的大小，帧频率是32

    if(video) //如果能创建CvVideoWriter对象则表明成功
    {
        cout<<"VideoWriter has created."<<endl;
    }

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;

    imgSub = it.subscribe("/uav_cam/image", 10, imageCallback);
            
    //cvReleaseVideoWriter(&video);

    ros::spin();
    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cout<< "get a Image."<<endl;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Image = cv_ptr->image;
    //imshow("debug img",Image);
    imwrite("debug img.jpg",Image);

    Mat Image1;
    Image1 = Image.clone();// 包括数据的深度复制，以防对mat数据的更改
    frame = cvCreateImage(cvSize(Image.cols,Image.rows),8,3); //根据实际进行初始化
    frame->imageData = (char*)Image1.data;

    n = cvWriteFrame(video,frame); //判断是否写入成功，如果返回的是1，表示写入成功

    if(n == 1)
    {
        cout<<"successfully write "<<n<<" frame."<<endl;
    }

    //cvShowImage("Video",frame); //显示视频内容的图片

    waitKey(1);
}

