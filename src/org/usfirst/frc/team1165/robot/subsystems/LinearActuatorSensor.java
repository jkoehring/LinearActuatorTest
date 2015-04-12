package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.RobotMap;
import org.usfirst.frc.team1165.robot.commands.ReportLinearActuatorPosition;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LinearActuatorSensor extends Subsystem
{
	private AnalogPotentiometer pot;
	
	private final static double scale = 12.0;		// the scale to apply to (multiply) the value returned by the POT

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	public LinearActuatorSensor()
	{
		pot = new AnalogPotentiometer(RobotMap.linearActuatorSensorPort, 12.0);
	}

	public void initDefaultCommand()
	{
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new ReportLinearActuatorPosition());
	}
	
	public double getPosition()
	{
		return pot.get();
	}
	
	public AnalogPotentiometer getPot()
	{
		return pot;
	}
	
	public void reportPosition()
	{
		SmartDashboard.putNumber(RobotMap.linearActuatorSensorKey, getPosition());
	}
}
