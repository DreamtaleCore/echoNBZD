/**
* Created by Dreamtale
* Mail to: dreamtalewind@gmail.com
* At 2015/12/6
*/
#include <Servo.h>
String Bdata = "";


void setup()
{
	Serial.begin(9600);
	Serial.println("Attention:");
}

void loop()
{
	// Input format: first_position, second_position, third_position
	// Intenger
	while (Serial.available())
	{
		Bdata += (char)Serial.read();
		delay(1);
	}

	if (Bdata.length() != 0)
	{
		Serial.println(Bdata);
                Serial.println();
	}

	Bdata = "";			// Clear Bdata
	delay(10);
}
