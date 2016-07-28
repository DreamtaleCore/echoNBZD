#include "ros/ros.h"
#include "../../include/stdHeader.h"

using namespace std;

void chatterCallback(const std_msgs::String::ConstPtr& msgs)
{
    ROS_INFO("I heard: [%s]", msgs->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("myFilter", 1000, chatterCallback);
//    ros::Subscriber sub = n.subscribe("colorDetec", 1000, chatterCallback);
    ros::spin();

    return 0;
}
