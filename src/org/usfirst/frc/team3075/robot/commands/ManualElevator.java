package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Robot;

import LibPurple.sensors.ConsoleJoystick;
import LibPurple.systems.DriveSystem3075;
import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualElevator extends Command 
{

	public ManualElevator() 
	{
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
	}
	
	@Override
	protected void initialize() 
	{
		// TODO Auto-generated method stub
	}
	
	@Override
	protected void execute() 
	{
		// TODO Auto-generated method stub
		Robot.elevator.setBigElevatorMasterMotor(Robot.oi.elevatorStick.getRawAxis(1));
		Robot.elevator.setSmallElevatorMotor(-Robot.oi.elevatorStick.getRawAxis(5));
	}
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
}
