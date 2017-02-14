#include <Subsystems/PigeonNav.h>
#include "WPILib.h"
#include "Subsystems/Drivetrain.h"
#include "Subsystems/Climber.h"
#include "OI.h"

class Robot: public frc::IterativeRobot {
	Command * autonomousCommand;
public:
	Robot();
	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
	static std::shared_ptr<Drivetrain> drivetrain;
	static std::shared_ptr<Climber> climber;
	static std::shared_ptr<PigeonNav> gyro;
	static std::shared_ptr<OI> oi;
private:

};
