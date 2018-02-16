package Autonomous;

import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.ActiveIntake;
import org.usfirst.frc.team3075.robot.commands.Wait;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MiddlePickUp extends CommandGroup
{
	public MiddlePickUp()
	{

		addSequential(Robot.driveSystem.driveStraightTrapizodial(1.67, false), 2.7);
		addSequential(new Wait(0.05));
		addSequential(Robot.driveSystem.turnAngleTrapizodial(24), 1.2);
		addSequential(new Wait(0.05));
		addParallel(new ActiveIntake(0.7, 0.7), 2);
		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.3, 0.25, 0.5), 2);
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(-0.3, false));
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(-1.1, false));
	}
}
