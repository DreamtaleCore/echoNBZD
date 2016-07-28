#include "ros/ros.h"
#include "../../include/stdHeader.h"
#define FILITER_1 500
#define ERROR_NUM -1
#define PI 3.14159256
#define DST_WIDTH 240
#define DST_HEIGHT 240

using namespace std;
ros::Publisher data_pubX, data_pubY;
int myData[5];
double lastData[2] = {0.0, 0.0};
int center[2] = {DST_WIDTH, DST_HEIGHT};
double history[2] = {0};

double P = 0.7;
double I = 0.0001;
double D = 0.09;

/**
 * @brief str2data
 * @param str, flag, data
 * @return dataSum
 * @author yunfei 2015/10/13
 */
int str2data(const char* str, char flag, int data[])
{
    ROS_INFO("I read source data: [%s]", str);
    // Init data
    int dataSum = 0, i = 0;
    for(i = 0; str[i] != '\0'; i++)
        if(str[i] == flag)
            data[dataSum ++] = 0;

    int j = 0;
    for(i = 0; i < str[i] != '\0'; i++)
    {
        if(str[i] == flag)
            j ++;
        else
        {
            data[j] = data[j] * 10 + str[i] - '0';
        }
    }

    return dataSum;
}

void PIDctrl(double src, double& dst, int posi)
{
    double change = src - lastData[posi];
    history[posi] = history[posi] + src;
    double error = P * src + I * history[posi] + D * change;
    lastData[posi] = dst = error;       // Update lastData
}

/**
 * @brief PosiCvtKernel
 * @param pixel  the raw data set
 * @param angle  the charming data we want
 * @param posi   0 -> x, 1 -> y
 */
void PosiCvtKernel(int pixel[], int& angle, int posi)
{
    const double k[2] = {330.0, 550.0}; // k is betaing...
    double h = 2 * k[posi] * (double)center[posi] / (double)pixel[2];
    cout << "srcPoint: "<< pixel[posi] << endl;
    double x = (double)(pixel[posi] - DST_HEIGHT);
    double inner = x / sqrt(x*x + h*h);
    cout << "Inner = " << inner << endl;
    double theta = inner + inner*inner*inner / 6
            + 3 * inner*inner*inner*inner*inner / 40;   // arcsin's taylor
    cout << "Count Angle " << theta / PI * 180 << endl;
    PIDctrl(theta, theta, posi);
    angle = (int)(theta / PI * 180);
    cout << "PID filter " << angle << endl << endl;
}

void chatterCallback(const std_msgs::String::ConstPtr& msgs)
{

    str2data(msgs->data.c_str(), ',', myData);

    int posiX = 0, angleX = 0;
    PosiCvtKernel(myData, angleX, posiX);

    int posiY = 1, angleY = 1;
    PosiCvtKernel(myData, angleY, posiY);

    // Store data and publish it
    std_msgs::UInt16 msgDataX;
//    std_msgs::UInt16 msgDataY;

    msgDataX.data = angleX;
//    msgDataY.data = angleY;

    data_pubX.publish(msgDataX);
//    data_pubY.s(msgDataY);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "posiCvt");
    ros::NodeHandle n;

    data_pubX = n.advertise<std_msgs::UInt16>("AngleX", 1000);

    ros::Subscriber sub = n.subscribe("myFilter", 1000, chatterCallback);

    ros::spin();

    return 0;
}

