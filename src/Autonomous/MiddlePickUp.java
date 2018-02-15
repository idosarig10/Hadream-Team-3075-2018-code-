package Autonomous;

import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.ActiveIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MiddlePickUp extends CommandGroup
{
	//TODO fix
	public MiddlePickUp()
	{

		addSequential(Robot.driveSystem.driveStraightTrapizodial(1.66, false), 3);
		addSequential(Robot.driveSystem.turnAngleTrapizodial(24), 1.33);
		addParallel(new ActiveIntake(0.7, 0.7), 3);
		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.3, 0.25, 0.5));
		addSequential(Robot.driveSystem.driveStraightTrapizodial(-0.3, false));
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(-1.1, false));
	}
}
