package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.RobotMap;
import org.usfirst.frc.team1165.robot.commands.OperateLinearActuatorWithJoystick;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LinearActuator extends Subsystem
{
	private CANTalon motor;
	public LinearActuator()
	{
		motor = new CANTalon(RobotMap.linearActuatorMotorChannel);
	}
	
	public void setSpeed(double speed)
	{
		SmartDashboard.putNumber(RobotMap.linearActuatorSpeedKey, speed);
		motor.set(speed);
	}
	
	public void idle()
	{
		setSpeed(0);
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand()
	{
		setDefaultCommand(new OperateLinearActuatorWithJoystick());
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
