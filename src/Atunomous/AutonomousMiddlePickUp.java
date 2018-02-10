package Atunomous;

import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousMiddlePickUp extends CommandGroup
{
	public AutonomousMiddlePickUp()
	{
		addSequential(Robot.driveSystem.driveStraightTrapizodial(1.7, false));
		addSequential(Robot.driveSystem.turnAngleTrapizodial(-10));
	}

}
