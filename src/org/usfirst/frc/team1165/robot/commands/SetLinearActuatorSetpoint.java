package org.usfirst.frc.team1165.robot.commands;

import org.usfirst.frc.team1165.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetLinearActuatorSetpoint extends Command
{
	private double setpoint;
	private String setpointKey;
	
	private SetLinearActuatorSetpoint()
	{
		requires(Robot.linearActuator);
	}

	public SetLinearActuatorSetpoint(double setpoint)
	{
		this();
		this.setpoint = setpoint;
	}

	public SetLinearActuatorSetpoint(String setpointKey)
	{
		this();
		this.setpointKey = setpointKey;
	}

	// Called just before this Command runs the first time
	protected void initialize()
	{
		if (setpointKey != null)
		{
			setpoint = SmartDashboard.getNumber(setpointKey);
		}
		
		Robot.linearActuator.enable();
		Robot.linearActuator.setSetpoint(setpoint);
		
		Robot.linearActuatorSensor.resetMinMax();
		Robot.linearActuator.resetMaxSpeed();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute()
	{
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished()
	{
		return Robot.linearActuator.onTarget();
	}

	// Called once after isFinished returns true
	protected void end()
	{
		Robot.linearActuator.disable();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted()
	{
		end();
	}
}
