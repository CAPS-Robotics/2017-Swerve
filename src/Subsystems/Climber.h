#ifndef Climber_H
#define Climber_H

#include <CANTalon.h>
#include "WPILib.h"
#include "Commands/Subsystem.h"

class Climber : public Subsystem {
private:
	CANTalon * climber1;
	CANTalon * climber2;
public:
	Climber();
	void SetClimberSpeed(double);
	void StopClimber();
	void InitDefaultCommand();
	CANTalon * GetClimber1();
	CANTalon * GetClimber2();
	double Get1Current();
	double Get2Current();
};

#endif  // Climber_H
