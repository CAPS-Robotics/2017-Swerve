#ifndef Vision_H
#define Vision_H

#include <CANTalon.h>
#include "WPILib.h"
#include "Commands/Subsystem.h"
#include <vector>

class Vision : public Subsystem {
private:
	std::shared_ptr<NetworkTable> table;
	std::vector<double> centerX;
	std::vector<double> centerY;
	std::vector<double> height;
	std::vector<double> width;
public:
	Vision();
	void Update();
	double GetCentralValue();
	void InitDefaultCommand();
};

#endif  // Vision_H
