package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.Robot;
import org.usfirst.frc.team1165.robot.RobotMap;
import org.usfirst.frc.team1165.robot.commands.OperateLinearActuatorWithJoystick;
import org.usfirst.frc.team1165.wpilibj.CANTalon;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * PID subsystem for the AndyMark am-3076 linear actuator.
 */
public class LinearActuatorPID extends PIDSubsystem
{
	private CANTalon motor;
	
	// PID parameters
	private final static double kP = 0.75;
	private final static double kI = 0.00;
	private final static double kD = 0;
	
	// Minimum and maximum allowable set point positions:
	private final static double minPosition = 0.5;
	private final static double maxPosition = 5.5;
	
	// Minimum and maximum allowable speeds:
	private final static double minSpeed = -0.74;
	private final static double maxSpeed = 0.74;
	
	// Tolerance for reaching set point:
	private final static double tolerance = 0.1;
	
	// Keeps track of maximum speed set by the PID controller.
	private double maxSetSpeed;

	// Constructor.
	public LinearActuatorPID()
	{
		// Set the parameters of the PID controller:
		super(kP, kI, kD);
		
		// Set ranges of PID controller:
        setAbsoluteTolerance(tolerance);
        setInputRange(minPosition, maxPosition);
        setOutputRange(minSpeed, maxSpeed);
		
        // Construct the motor that drives the actuator.
		motor = new CANTalon(RobotMap.linearActuatorMotorChannel);

		// Let's show everything on the LiveWindow:
        LiveWindow.addActuator("Actuator", "Motor", motor);
        LiveWindow.addSensor("Actuator", "Pot", Robot.linearActuatorSensor.getPot());
        LiveWindow.addActuator("Actuator", "PID", getPIDController());
	}

	/**
	 * Set a default command that allows us to control the actuator via the
	 * joystick when it is not being controlled by the PID controller.
	 */
	public void initDefaultCommand()
	{
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new OperateLinearActuatorWithJoystick());
	}
	
	public void resetMaxSpeed()
	{
		maxSetSpeed = 0;
	}

	/**
	 * Called by the PID controller to get the current input value.
	 * Returns the current value from the analog potentiometer
	 * indicating the position of the actuator.
	 */
	protected double returnPIDInput()
	{
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
		return Robot.linearActuatorSensor.getPosition();
	}

	/**
	 * Sets the motor speed between -1 and 1.
	 */
	public void setSpeed(double speed)
	{
		SmartDashboard.putNumber(RobotMap.linearActuatorSpeedKey, speed);
		motor.set(speed);
		if (Math.abs(speed) > Math.abs(maxSetSpeed))
		{
			maxSetSpeed = speed;
		}
		SmartDashboard.putNumber(RobotMap.linearActuatorSpeedMaxKey, maxSetSpeed);
	}
	
	/**
	 * Called by the PID controller with its current calculated output value,
	 * which is used to set the motor direction and speed.
	 */
	protected void usePIDOutput(double output)
	{
		setSpeed(output);
	}
}
