//include system in/out libraries
#include <iostream>

//include ros libraries
#include <ros/ros.h>

//include messege libraries
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Vector3.h"

//include opencv libraries
#include <opencv2/opencv.hpp>

//include ros transport&bridge libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "../../include/stdHeader.h"

///<可以修改的区域>
#define BUFSIZE 512
#define BUAD B9600  //注意前面有个B
///</可以修改的区域>
#define TRUE 1

using namespace std;

ros::Publisher data_pub;
int myData[5], lastData[5];
std_msgs::String pubData;


void chatterCallbackColor(const std_msgs::String::ConstPtr& msgs)
{
    char* cmd_data = (char*)msgs->data.c_str();
    ROS_INFO("I get the Color data: [%s]\n", cmd_data);
    std_msgs::String msg;
    stringstream ss;
    ss << "10|" << cmd_data << "|01";
    msg.data = ss.str();
    data_pub.publish(msg);
}

void chatterCallbackAruco(const geometry_msgs::Vector3 cmd_data)
{
    ROS_INFO("I get the Aruco data: [%f, %f, %f]\n", cmd_data.x, cmd_data.y, cmd_data.z);
    // Publish data here~
        std_msgs::String msg;
        stringstream ss;
        ss << "00|" << cmd_data.x <<","
               << cmd_data.y << "," << cmd_data.z << "|00";
        msg.data = ss.str();
        data_pub.publish(msg);
        //msg = send_data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CvtCommand");
    ros::NodeHandle n;

    // Binding the da
    data_pub = n.advertise<std_msgs::String>("CvtCommand", 400);

    ros::Subscriber subColor = n.subscribe("optColorTrack", 400, chatterCallbackColor);
    ros::Subscriber subAruco = n.subscribe("optArucoTrack", 400, chatterCallbackAruco);

    ros::spin();

    return 0;
}


//==============================================

// the content of the table

// +--------+----------+----------------+----------+
// | itemID | itemName | objectCategory | position |
// +--------+----------+----------------+----------+ 
