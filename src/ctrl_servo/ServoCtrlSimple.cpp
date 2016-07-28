#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <Servo.h>
#include <std_msgs/UInt16.h>

#define INIT_POSI  80
#define LEV_RANGE  90

Servo servoX;
//Servo servoY;

int lastAngle[2] = {INIT_POSI, INIT_POSI};
ros::NodeHandle  nh;

void initServo()
{
    servoX.attach(9);
    servoX.write(INIT_POSI);

//    servoY.attach(10);
//    servoY.write(INIT_POSI);

}

/**
 * @brief justMov
 * @param servo
 * @param angle
 * @param posi
 */
void justMov(Servo servo, int angle, int posi)
{
    // aviod error data
    if (angle < 1)
        angle = 1;
    if (angle > 179)
        angle = 179;

    if (angle > lastAngle[posi])
    {
        for (int ii = lastAngle[posi]; ii < angle; ii += 1)
        {
            servo.write(ii);
            delay(2);
        }
    }
    else
    {
        for (int ii = lastAngle[posi]; ii > angle; ii -= 1)
        {
            servo.write(ii);
            delay(2);
        }
    }
    lastAngle[posi] = angle;
}


void servo_cbX(const std_msgs::UInt16& cmd_msg){
    int dPosi = cmd_msg.data;
    int posix = 0;
    justMov(servoX, INIT_POSI - dPosi, posix);
    digitalWrite(13, HIGH - digitalRead(13));  //toggle
    delay(100);
}

//void servo_cbY(const std_msgs::UInt16& cmd_msg){
//    int dPosi = cmd_msg.data;
//    int posiy = 1;
//    justMov(servoX, INIT_POSI + dPosi, posiy);
//    digitalWrite(13, HIGH - digitalRead(13));  //toggle
//    delay(100);
//}

ros::Subscriber<std_msgs::UInt16> subX("AngleX", servo_cbX);
//ros::Subscriber<std_msgs::UInt16> subY("AngleY", servo_cbY);

void setup()
{
    pinMode(13, OUTPUT);
    initServo();

    nh.initNode();
    nh.subscribe(subX);
//    nh.subscribe(subY);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}
