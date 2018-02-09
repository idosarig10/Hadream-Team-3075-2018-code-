package Atunomous;

import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousLeft extends CommandGroup{
	public AutonomousLeft()
	{
		addSequential(Robot.driveSystem.driveStraightTrapizodial(5.20, false));
		addSequential(Robot.driveSystem.driveArc(0.76, 180, true));
	}
}
