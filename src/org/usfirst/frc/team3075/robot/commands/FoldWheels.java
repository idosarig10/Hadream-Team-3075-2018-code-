package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FoldWheels extends Command {

    public FoldWheels() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.intake.wheelsCylinder.ToggleCommand();
    	if(Robot.intake.isFolded)
    	{
    		Robot.intake.isFolded = false;
    	}
    	else
    	{
    		Robot.intake.isFolded = true;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
