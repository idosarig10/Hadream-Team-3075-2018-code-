package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoSetSmallElevator extends Command 
{
	private double setpoint;
	
    public AutoSetSmallElevator(double setpoint) 
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.elevator);
    	this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.elevator.setSmallElevatorSetpoint(setpoint);
    	Robot.elevator.activeSmallElevatorPID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
    	return Robot.elevator.smallElevatorOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() 
    {
    	Robot.elevator.disableSmallElevatorPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	end();
    }
}
