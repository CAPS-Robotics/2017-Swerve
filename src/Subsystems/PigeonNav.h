#ifndef PigeonNav_H
#define PigeonNav_H

#include <PigeonImu.h>
#include "WPILib.h"
#include "Commands/Subsystem.h"

class PigeonNav : public Subsystem {
private:
	PigeonImu * gyro;
public:
	PigeonNav();
	double GetHeading();
	void ResetHeading();
	void InitDefaultCommand();
};

#endif  // PigeonNav_H
