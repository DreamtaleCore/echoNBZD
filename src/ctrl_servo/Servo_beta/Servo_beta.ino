#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>

#define INIT_POSI  90
#define LEV_RANGE  90
#define FIG_WIDTH  320
#define FIG_HEIGHT 240

Servo servoX;
Servo servoY;
String Bdata = "";
int val, last_valX, last_valY;
int myData[5] = {FIG_WIDTH, FIG_HEIGHT, 0, 0}; //Init data;
ros::NodeHandle  nh;

void initServo()
{
    servoX.attach(9);
    servoX.write(INIT_POSI);

    servoY.attach(10);
    servoY.write(INIT_POSI);

    last_valX = last_valY = INIT_POSI;
}

/**
 * @brief justMov
 * @param posi
 * @return last_val
 */
int justMov(Servo servo, int posi, int last_val)
{
	// aviod error data
	if (posi < 1)
		posi = 1;
	if (posi > 179)
		posi = 179;

    Serial.print("The last value = "); Serial.println(last_val);
	Serial.print("Current  value = "); Serial.println(posi);
	Serial.println();

    if (posi > last_val)
	{
        for (int ii = last_val; ii < posi; ii += 1)
		{
			servo.write(ii);
			delay(2);
		}
	}
	else
	{
        for (int ii = last_val; ii > posi; ii -= 1)
		{
			servo.write(ii);
			delay(2);
		}
	}
    return posi;
}

/**
 * @brief levelXhg, get the level change data from myData
 * @param data
 * @return int
 */
int levelXhg(int data, int type)
{
  if(type == 1)
    return -(FIG_WIDTH - data) * LEV_RANGE / FIG_WIDTH + INIT_POSI;
  else
    return -(FIG_HEIGHT - data) * LEV_RANGE / FIG_HEIGHT + INIT_POSI;
}

void servo_cbX(const std_msgs::UInt16& cmd_msg){
    int levMov = levelXhg(cmd_msg.data, 1);
    last_valX = justMov(servoX, levMov, last_valX);        // update the last_valX
    digitalWrite(13, HIGH - digitalRead(13));  //toggle
    delay(500);
}

void servo_cbY(const std_msgs::UInt16& cmd_msg){
    int levMov = levelXhg(cmd_msg.data, 2);
    last_valY = justMov(servoY, levMov, last_valY);        // update the last_valY
    digitalWrite(13, HIGH - digitalRead(14));  //toggle
    delay(500);
}

ros::Subscriber<std_msgs::UInt16> subX("myFilterX", servo_cbX);
ros::Subscriber<std_msgs::UInt16> subY("myFilterY", servo_cbY);

void setup()
{
    pinMode(13, OUTPUT);
	initServo();

    nh.initNode();
    nh.subscribe(subX);
    nh.subscribe(subY);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}

