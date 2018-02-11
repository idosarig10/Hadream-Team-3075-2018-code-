package Atunomous;

import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.ActiveIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousMiddlePickUp extends CommandGroup
{
	//TODO fix
	public AutonomousMiddlePickUp()
	{
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(1.7, false));
//		addSequential(Robot.driveSystem.turnAngleTrapizodial(-20));
//		addParallel(new ActiveIntake(0.7, 0.7), 2.5);
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.5, 0.25, 0.5));
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.4, false));
		addSequential(Robot.driveSystem.driveStraightTrapizodial(1.62, false));
		addSequential(Robot.driveSystem.turnAngleTrapizodial(30), 1.33);
		addParallel(new ActiveIntake(0.7, 0.7), 3);
		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.4, 0.25, 0.5));
		addSequential(Robot.driveSystem.driveStraightTrapizodial(-1.1, false));
	}
}
