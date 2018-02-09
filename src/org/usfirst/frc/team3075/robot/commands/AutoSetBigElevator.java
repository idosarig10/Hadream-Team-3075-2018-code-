package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoSetBigElevator extends Command 
{
	private double setpoint;
	
    public AutoSetBigElevator(double setpoint) 
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.elevator);
    	this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.setBigElevatorSetpoint(setpoint);
    	Robot.elevator.activeBigElevatorPID();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.elevator.setBigElevatorMasterMotor(1);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
    	return Robot.elevator.bigElevatorOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() 
    {
    	Robot.elevator.disableBigElevatorPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	end();
    }
}
