#include "ros/ros.h"
#include "../../include/stdHeader.h"

using namespace std;

void chatterCallback(const std_msgs::UInt16& msgs)
{
    cout << "The angle is  " << msgs.data << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("AngleX", 1000, chatterCallback);
//    ros::Subscriber sub = n.subscribe("colorDetec", 1000, chatterCallback);
    ros::spin();

    return 0;
}
