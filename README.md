# 2017-Swerve

This is the code that went into the drivetrain for Team 2410's Swerve robot for the Steamworks game. With our robot, we were able to place 4-5 gears per match, plus a perfected autonomous gear for any peg at worlds this year. We made it to worlds for the first time in 5 years and won a regional (Iowa) for the first time in 7 years. In addition, our robot is highway usable we discovered as it drove in the Stilwell Fourth of July Parade.

This code is only for the swerve drivetrain, to provide an extendible system for future members and people using the Andymark Swerve n Steer modules who need some code for a starting point.

This code was inspired by that of many other swerve teams, including the Baxter Bomb Squad, FRC Team 16.

##  System

Our code used the Command-Based robot framework, which we found worked a lot better than using a single file.

To control the robot we utilized 1 Logitech Extreme 3D joystick. With it, we were able to map the rotation of the bot to the twist axis of the controller. Also we used the throttle on the joystick as an actual throttle if we needed to slow the bot down.

We used the Andymark Swerve-n-Steer modules. We found they did work quite well when used with the analog encoders that are made for them. The digital encoders were not precise for us.

We also utilized the Pigeon Nav Board for our gyro sensor. We had it plugged into our climber's Talon SRX. In the PigeonNav subsystem, you have to supply a TalonSRX as a parameter to the class in order to use it.

We used GRIP to process a Microsoft LifeCam on the driver station rather than using an onboard computer. It worked quite well over!

We hope this can help people! 
