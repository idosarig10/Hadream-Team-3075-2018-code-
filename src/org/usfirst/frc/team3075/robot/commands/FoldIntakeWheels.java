package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FoldIntakeWheels extends Command {

	private boolean forward;
	
    public FoldIntakeWheels(boolean forward) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    	this.forward = forward;
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.intake.setSolenoid(forward ? Value.kForward : Value.kReverse);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
