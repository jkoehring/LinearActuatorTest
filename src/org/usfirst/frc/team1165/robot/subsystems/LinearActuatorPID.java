package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.Robot;
import org.usfirst.frc.team1165.robot.RobotMap;
import org.usfirst.frc.team1165.robot.commands.OperateLinearActuatorWithJoystick;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LinearActuatorPID extends PIDSubsystem
{
	private CANTalon motor;
	private final static double kP = 4.0;
	private final static double kI = 0.04;
	private final static double kD = 0;
	
	private final static double minPosition = 0.5;
	private final static double maxPosition = 5.4;
	
	private final static double minSpeed = -0.2;
	private final static double maxSpeed = 0.2;

	// Initialize your subsystem here
	public LinearActuatorPID()
	{
		super(kP, kI, kD);
		
		motor = new CANTalon(RobotMap.linearActuatorMotorChannel);
		
        setAbsoluteTolerance(0.005);
        setInputRange(minPosition, maxPosition);
        setOutputRange(minSpeed, maxSpeed);

		// Let's show everything on the LiveWindow
        //LiveWindow.addActuator("Actuator", "Motor", (LiveWindowSendable) motor);
        //LiveWindow.addSensor("Actuator", "Pot", Robot.linearActuatorSensor.getPot());
        //LiveWindow.addActuator("Actuator", "PID", getPIDController());
	
	}

	public void initDefaultCommand()
	{
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new OperateLinearActuatorWithJoystick());
	}

	protected double returnPIDInput()
	{
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
		return Robot.linearActuatorSensor.getPosition();
	}

	public void setSpeed(double speed)
	{
		SmartDashboard.putNumber(RobotMap.linearActuatorSpeedKey, speed);
		motor.set(speed);
	}
	
	protected void usePIDOutput(double output)
	{
		setSpeed(output);
	}
}
