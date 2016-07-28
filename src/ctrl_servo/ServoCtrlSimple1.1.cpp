#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <Servo.h>
#include <std_msgs/UInt32.h>

#define INIT_POSI  80
#define LEV_RANGE  90

Servo servoX;
Servo servoY;

int lastAngle[2] = {INIT_POSI, INIT_POSI};
ros::NodeHandle  nh;

void initServo()
{
    servoX.attach(9);
    servoX.write(INIT_POSI);

    servoY.attach(10);
    servoY.write(INIT_POSI);

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


void servo_cb(const std_msgs::UInt32& cmd_msg){
    long long dPosi = cmd_msg.data;
    int anglex = 0, angley = 0;
    int posix = 0, posiy = 1;
    anglex = (dPosi & 0xffff0000) >> 16;
    angley = (dPosi & 0xffff);
    justMov(servoX, INIT_POSI - anglex, posix);
    justMov(servoY, INIT_POSI - angley, posiy);
    digitalWrite(13, HIGH - digitalRead(13));  //toggle
    delay(100);
}

ros::Subscriber<std_msgs::UInt32> sub("Angle", servo_cb);

void setup()
{
    pinMode(13, OUTPUT);
    initServo();

    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}
