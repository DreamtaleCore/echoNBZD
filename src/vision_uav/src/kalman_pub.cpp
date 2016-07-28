#include <iostream>
#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include "std_msgs/Float32.h"

using namespace cv;
using namespace ros;
using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"pub");
    ros::NodeHandle nh;

    ros::Publisher pub;
    pub = nh.advertise <std_msgs::Float32>("kalman_sub",200);

    std_msgs::Float32 outMsg;
    Mat temp(1,1,CV_32F);

    float count;
    float tmp;
    bool _bool = false;

    tmp = 0;
    count = 1;

    while (ros::ok())
    {

        // generate measuremen
        tmp = 0;
        // measurementNoiseCov = 0.1;
        randn( temp, Scalar::all(0), Scalar::all(0.1));
        tmp += temp.at<float>(0,0);
        tmp += count;

        outMsg.data = tmp;
        pub.publish( outMsg );

        cout<<tmp<<endl;

        // 0--50--0
        if(count <= 50 && _bool == false)
        {
            count += 0.1;
            if(count > 49)
                _bool = true;
        }
        else
        {
            count -= 0.1;
            if(count < 1)
                _bool = false;
        }

        ros::spinOnce();
        ros::Rate loop_rate(15);
        loop_rate.sleep();

    }

    return 0;

}
