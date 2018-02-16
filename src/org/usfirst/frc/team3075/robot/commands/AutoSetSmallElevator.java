package org.usfirst.frc.team3075.robot.commands;

import java.io.FileWriter;
import java.util.logging.FileHandler;

import org.usfirst.frc.team3075.robot.Robot;

import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoSetSmallElevator extends Command {
	
	private double setpoint;
	FileWriter handle;

    public AutoSetSmallElevator(double setpoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(Robot.elevator);
    	this.setpoint = setpoint;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	handle = Utils.initialiseCSVFile("/graphs/elevator");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	Robot.elevator.setSmallElevatorSetpoint(setpoint);
    	Robot.elevator.activeSmallElevatorPID();
    	double[] params = {Robot.elevator.smallElevatorPID.getSetpoint(), Robot.elevator.smallElevatorMotor.get()};
    	Utils.addCSVLine(handle, params);
  	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    	Robot.elevator.disableSmallElevatorPID();
    	Utils.closeCSVFile(handle);
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    	end();
    	
    }
}
