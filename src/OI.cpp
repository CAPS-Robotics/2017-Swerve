#include "OI.h"

#include <WPILib.h>
#include "Commands/Drivetrain/SetArcadeMode.h"

OI::OI() {
	joy1 = new Joystick(0);
	button1 = new JoystickButton(joy1, 1);
	button2 = new JoystickButton(joy1, 2);
	button3 = new JoystickButton(joy1, 3);
	button4 = new JoystickButton(joy1, 4);
	button5 = new JoystickButton(joy1, 5);
	button6 = new JoystickButton(joy1, 6);
	button7 = new JoystickButton(joy1, 7);
	button8 = new JoystickButton(joy1, 8);
	button9 = new JoystickButton(joy1, 9);
	button10 = new JoystickButton(joy1, 10);
	button11 = new JoystickButton(joy1, 11);
	button12 = new JoystickButton(joy1, 12);

	button1->WhileHeld(new SetArcadeMode());
}

float OI::GetX() {
	return this->applyDeadzone(joy1->GetRawAxis(0), 0.15);
}

float OI::GetY() {
	return this->applyDeadzone(-joy1->GetRawAxis(1), 0.15);
}

float OI::GetTwist() {
	return this->applyDeadzone(joy1->GetRawAxis(2), 0.70);
}

float OI::GetSlider() {
	return joy1->GetRawAxis(3);
}

float OI::applyDeadzone(float val, float deadzone) {
	if (fabs(val) <= deadzone) {
		return 0;
	}
	return val;
}
