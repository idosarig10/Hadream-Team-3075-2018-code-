package Atunomous;

import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousLeftScale extends CommandGroup
{
	public AutonomousLeftScale()
	{
		addSequential(Robot.driveSystem.driveStraightTrapizodial(7, false));
	}

}
