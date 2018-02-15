package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoSetElevators extends Command {
	
	private double bigElevatorSetpoint;
	private double smallElevatorSetpoint;


    public AutoSetElevators(double bigElevatorSetpoint, double smallElevatorSetpoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.elevator);
    	this.bigElevatorSetpoint = bigElevatorSetpoint;
    	this.smallElevatorSetpoint = smallElevatorSetpoint;
    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	Robot.elevator.setBigElevatorSetpoint(this.bigElevatorSetpoint);
    	Robot.elevator.setSmallElevatorSetpoint(this.smallElevatorSetpoint);
    	Robot.elevator.activeBigElevatorPID();
    	Robot.elevator.activeSmallElevatorPID();
    	
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
    	
    	Robot.elevator.disableBigElevatorPID();
    	Robot.elevator.disableSmallElevatorPID();
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    	end();
    	
    }
}
