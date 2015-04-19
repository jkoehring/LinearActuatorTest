package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.RobotMap;
import org.usfirst.frc.team1165.robot.commands.ReportLinearActuatorPosition;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LinearActuatorSensor extends Subsystem
{
	private AnalogPotentiometer pot;
	
	private final double scale = 12.0;		// the scale to apply to (multiply) the value returned by the POT
	
	private double minPosition;
	private double maxPosition;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	public LinearActuatorSensor()
	{
		pot = new AnalogPotentiometer(RobotMap.linearActuatorSensorPort, scale);
		resetMinMax();
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
	
	public void resetMinMax()
	{
		synchronized(this)
		{
			minPosition = Double.MAX_VALUE;
			maxPosition = Double.MIN_VALUE;
		}
	}
	
	public void reportPosition()
	{
		synchronized (this)
		{
			double position = getPosition();
			minPosition = Math.min(minPosition, position);
			maxPosition = Math.max(maxPosition, position);
			SmartDashboard.putNumber(RobotMap.linearActuatorSensorKey, position);
			SmartDashboard.putNumber(RobotMap.linearActuatorSensorMinKey, minPosition);
			SmartDashboard.putNumber(RobotMap.linearActuatorSensorMaxKey, maxPosition);
		}
	}
}
