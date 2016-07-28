/**
 * We can get different params
 * from the "myFilter" node and using these
 * params we can control the servo to make the
 * camara get a better picture
 * @author yunfei 2015/10/14
*/
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo;

int myData[5];

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

void servo_cb(std_msgs::String::ConstPtr& cmd_msg){
    int dataSum = str2data(cmd_msgs->data.c_str(), ',', myData);
	servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
	digitalWrite(13, HIGH - digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> sub("myFilter", servo_cb);

void setup(){
	pinMode(13, OUTPUT);

	nh.initNode();
	nh.subscribe(sub);

	servo.attach(9); //attach it to pin 9
}

void loop(){
	nh.spinOnce();
	delay(1);
}
