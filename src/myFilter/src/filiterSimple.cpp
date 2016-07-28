#include "ros/ros.h"
#include "../../include/stdHeader.h"
#define FILITER_1 500

using namespace std;

ros::Publisher data_pub;
int myData[5], lastData[5];
std_msgs::String pubData;

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

//    printf("dataSum = %d, %d,%d,%d,%d\n",
//           dataSum, data[0], data[1], data[2], data[3]);

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
   for(i = 0; i < dataSum; i++)
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

    if(myFiliterSimple(myData, lastData, dataSum) < FILITER_1)
    {
        printf("I get a better data: [ ");
        for(int i = 0; i < dataSum; i++)
        {
            printf("%d ", myData[i]);
        }
        printf("]\n");
        // Store data and publish it
        stringstream ss;
        ss << myData[0] <<","<< myData[1] <<","
               << myData[2] <<","<< myData[3] <<",";
        pubData.data = ss.str();
        data_pub.publish(pubData);
    }
    else
        ROS_INFO("I detect some error: [%d]\n", myData[0]);
    copyData(lastData, myData, dataSum);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myFilter");
    ros::NodeHandle n;


    data_pub = n.advertise<std_msgs::Int16MultiArray>("myFilter", 1000);

    ros::Subscriber sub = n.subscribe("movDetec", 1000, chatterCallback);

    ros::spin();

    return 0;
}


//==============================================

// the content of the table

// +--------+----------+----------------+----------+
// | itemID | itemName | objectCategory | position |
// +--------+----------+----------------+----------+ 
