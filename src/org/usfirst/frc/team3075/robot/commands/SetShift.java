package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.subsystems.Chassis.Shift;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetShift extends InstantCommand {

	private Shift shift;

	public SetShift(Shift s)
	{
		this.shift = s;
	}

	// Called just before this Command runs the first time
	protected void initialize()
	{
		Robot.driveSystem.setShifter(this.shift);
	}
}