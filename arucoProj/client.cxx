#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <vector>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace std;

enum uavCtrlSignal
{
    uavTakeOff = 0,
    uavLanding,
    uavHangSlient,
    uavMovFront,
    uavMovBack,
    uavMovLeft,
    uavMovRight,
    uavTurnLeft,
    uavTurnRight,
    uavForceMannul
};

class CmdParser
{
    std_msgs::UInt32::ConstPtr cmd;
    vector<int> cmds;

public:
    CmdParser(std_msgs::UInt32::ConstPtr _cmd) : cmd(_cmd)
    {}

    int operator [](int idx)
    {

    }
};

void coreLogicCallback(const std_msgs::UInt32::ConstPtr& _cmd)
{
    int cmd = _cmd.get();
    switch (cmd) {
    case uavMovFront:

        break;
    case uavMovBack:

        break;
    case uavMovLeft:

        break;
    case uavMovRight:

        break;

    case uavTakeOff:

        break;
    case uavLanding:

        break;
    case uavForceMannul:

        break;
    case uavTurnLeft:       // Disabled
    case uavTurnRight:
        break;

    default:
        break;
    }
}

using namespace DJI::onboardSDK;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "onboard_sdk");
    ros::NodeHandle nh;

    ros::Subscriber sub;

    sub = nh.subscribe("core_logic", 1000, coreLogicCallback);

    ros::spin();

    return 0;
}

