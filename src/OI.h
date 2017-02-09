#ifndef OI_H
#define OI_H

#include "WPILib.h"

class OI
{
private:
	Joystick * joy1;
	JoystickButton * button1;
	JoystickButton * button2;
	JoystickButton * button3;
	JoystickButton * button4;
	JoystickButton * button5;
	JoystickButton * button6;
	JoystickButton * button7;
	JoystickButton * button8;
	JoystickButton * button9;
	JoystickButton * button10;
	JoystickButton * button11;
	JoystickButton * button12;
public:
	OI();
	float GetX();
	float GetY();
	float GetTwist();
	float GetSlider();
	float applyDeadzone(float val, float deadzone);
};

#endif
