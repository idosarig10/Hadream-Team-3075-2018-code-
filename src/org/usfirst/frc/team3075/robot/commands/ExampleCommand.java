<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> latesttryupdate
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3075.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3075.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
<<<<<<< HEAD
=======
package org.usfirst.frc.team3075.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team3075.robot.Robot;

/**
 *
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
>>>>>>> latesttryupdate
 */
public class ExampleCommand extends Command {
	public ExampleCommand() {
		// Use requires() here to declare subsystem dependencies
<<<<<<< HEAD
<<<<<<< HEAD
		requires(Robot.kExampleSubsystem);
=======
		requires(Robot.exampleSubsystem);
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
		requires(Robot.kExampleSubsystem);
>>>>>>> latesttryupdate
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}