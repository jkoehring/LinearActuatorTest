package org.usfirst.frc.team1165.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap
{
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
    public static final int joystickPort0					= 0;
    
    public static final int linearActuatorMotorChannel		= 0;
    
    public static final int linearActuatorSensorPort		= 0;
    
    public static final String linearActuatorSpeedKey		= "Linear Actuator Speed";
    public static final String linearActuatorSpeedMaxKey	= "Linear Actuator Speed Max";
    public static final String linearActuatorSensorKey		= "Linear Actuator Sensor";
    public static final String linearActuatorSensorMinKey	= "Linear Actuator Sensor Min";
    public static final String linearActuatorSensorMaxKey	= "Linear Actuator Sensor Max";
    public static final String linearActuatorSetpointKey	= "Linear Actuator Setpoint";

}
