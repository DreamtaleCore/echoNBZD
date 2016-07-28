/**
  * Read data from the point serial commmunicate port
  */

#include "commCtrl.h"
#include "stdHeaders.h"
#include "ros/ros.h"

//==============================================

// the content of the data packet

// +------------+----------+----------------+------------+
// | packetHead | itemName | objectCategory | packetTail |
// +------------+----------+----------------+------------+

char dev[13] = "/dev/ttyUSB0";
std_msgs::String pubData;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serialRead");
    ros::NodeHandle nh;

    ros::Publisher publisher;
    publisher = nh.advertise<std_msgs::String>("serialRead", 1000);

    string data;

    ros::Rate loopRate(5);

    while(nh.ok())
    {
        int ret = commRead(data, dev);
        if(ret == -1)
        {
            cout << "Assert the serial port failed!" << endl;
            break;
        }
        pubData.data = data;
        ROS_INFO("I've read the data: [%s]\n", data.c_str());
        publisher.publish(pubData);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
