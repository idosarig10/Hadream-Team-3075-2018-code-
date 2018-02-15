package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Robot;

import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ActiveIntake extends Command
{

	private double conveyorSpeed;
	private double wheelsSpeed;
	
    public ActiveIntake(double conveyorSpeed, double wheelsSpeed) 
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    	this.conveyorSpeed = conveyorSpeed;
    	this.wheelsSpeed = wheelsSpeed;
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.intake.rightServo.setAngle(-90);
    	Robot.intake.leftServo.setAngle(90);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	Robot.intake.setConveyorSpeed(conveyorSpeed);
    	if(!Robot.intake.isFolded)
    	{
	    	Robot.intake.setLeftWheelSpeed(wheelsSpeed);
	    	Robot.intake.setRightWheelSpeed(-wheelsSpeed);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() 
    {
    	Robot.intake.setConveyorSpeed(0);
    	Robot.intake.setLeftWheelSpeed(0);
    	Robot.intake.setRightWheelSpeed(0);
    	Robot.intake.leftServo.setAngle(0);
    	Robot.intake.rightServo.setAngle(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	end();
    }
}
