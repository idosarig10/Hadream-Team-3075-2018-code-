package Atunomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousRightScale extends CommandGroup
{
	public AutonomousRightScale()
	{
		addSequential(Robot.driveSystem.driveStraightRelativeTolerance(7, 0.01));
		addSequential(Robot.driveSystem.turnAngleTrapizodial(90));
		addSequential(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
		addSequential(new ReleaseCube(), 1);
	}
}
