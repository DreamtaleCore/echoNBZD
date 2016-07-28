#include "ros/ros.h"
#include "../../include/stdHeader.h"
#define FILITER_1 500
#define ERROR_NUM -1

using namespace std;

ros::Publisher data_pubX;
ros::Publisher data_pubY;
int myData[5], lastData[5];

/**
 * @brief str2data
 * @param str, flag, data
 * @return dataSum
 * @author yunfei 2015/10/13
 */
int str2data(const char* str, char flag, int data[])
{
    ROS_INFO("I read source data: [%s]", str);
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

/**
 * @brief myFiliterSimple
 * @param curr_data, last_data, dataSum
 * @return distance between curr_data and last_data
 * @author yunfei 2015/10/13
 */
int myFiliterSimple(int curr_data[], int last_data[], int dataSum)
{
   int i, errDetec = 0;
   // use the image wide and height to varify
   // this result wether right or not
   for(i = 2; i < dataSum; i++)
       errDetec += (curr_data[i] - last_data[i])
                   * (curr_data[i] - last_data[i]);
   printf("errDis = %d\n", errDetec);
   return errDetec;
}

void copyData(int dst[], int src[], int dataSum)
{
    for(int i = 0; i < dataSum; i++)
        dst[i] = src[i];
}

void chatterCallback(const std_msgs::String::ConstPtr& msgs)
{

    int dataSum = str2data(msgs->data.c_str(), ',', myData);
    // Store data and publish it
    std_msgs::UInt16 msgDataX;
    std_msgs::UInt16 msgDataY;

    if(myFiliterSimple(myData, lastData, dataSum) < FILITER_1)
    {
        printf("I get a better data: [ ");
        for(int i = 0; i < dataSum; i++)
            printf("%d ", myData[i]);
        printf("]\n");

        msgDataX.data = myData[0];
        msgDataY.data = myData[1];

    }
    else
    {
        ROS_INFO("I detect some error: [%d]\n", myData[0]);
        msgDataX.data = ERROR_NUM;
        msgDataY.data = ERROR_NUM;
    }
    data_pubX.publish(msgDataX);
    data_pubY.publish(msgDataY);
    copyData(lastData, myData, dataSum);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myFilter");
    ros::NodeHandle n;

    data_pubX = n.advertise<std_msgs::UInt16>("myFilterX", 1000);
    data_pubY = n.advertise<std_msgs::UInt16>("myFilterY", 1000);

    ros::Subscriber sub = n.subscribe("faceDetec", 1000, chatterCallback);

    ros::spin();

    return 0;
}

