#include <Subsystems/Vision.h>
#include "../RobotMap.h"
#include <CANTalon.h>
#include <vector>

Vision::Vision() : Subsystem("Vision") {
	table = NetworkTable::GetTable("GRIP/AllDemContours");
	centerX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	centerY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
	height = table->GetNumberArray("height", llvm::ArrayRef<double>());
	width = table->GetNumberArray("width", llvm::ArrayRef<double>());
}

void Vision::Update() {
	centerX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	centerY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
	height = table->GetNumberArray("height", llvm::ArrayRef<double>());
	width = table->GetNumberArray("width", llvm::ArrayRef<double>());
}

double Vision::GetCentralValue() {
	this->Update();
	if (centerX.size() == 2) {
		double id = centerX[0] > centerX[1] ? 0 : 1;
		double leftEdge = centerX[1 - id] + width[1 - id] / 2;
		double rightEdge = centerX[id] - width[id] / 2;
		return (leftEdge + rightEdge) / 2.0;
	} else if (centerX.size() == 1) {
		return 320 - centerX[0];
	} else {
		return centerX.size();
	}
}

void Vision::InitDefaultCommand() {

}
