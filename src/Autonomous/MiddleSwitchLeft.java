package Autonomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.ActiveIntake;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MiddleSwitchLeft extends CommandGroup
{
	public MiddleSwitchLeft() 
	{
		addSequential(Robot.driveSystem.turnAngleTrapizodial(-60), 2.4);
		addSequential(Robot.driveSystem.driveStraightTrapizodial(2.4, false));
		addSequential(Robot.driveSystem.driveArcMpValues(0.82, 140, true, Constants.arcMPValue));
//		addParallel(new ActiveIntake(0.5, 0.5), 2.5);
//		addSequential(Robot.driveSystem.turnAngleTrapizodial(135));
//		addParallel(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.9, false));
//		addSequential(new ReleaseCube(), 1);
		
	}

}
