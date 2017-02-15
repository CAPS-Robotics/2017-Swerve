#ifndef PigeonNav_H
#define PigeonNav_H

#include <PigeonImu.h>
#include "WPILib.h"
#include "Commands/Subsystem.h"

class PigeonNav : public Subsystem {
private:
	PigeonImu * gyro;
	double * ypr;
public:
	PigeonNav();
	double GetHeading();
	double GetAngularRate();
	void ResetHeading();
	void InitDefaultCommand();
};

#endif  // PigeonNav_H
