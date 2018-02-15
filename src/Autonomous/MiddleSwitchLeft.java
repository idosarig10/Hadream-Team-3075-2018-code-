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
		addSequential(Robot.driveSystem.turnAngleTrapizodial(-90));
		addParallel(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
		addSequential(Robot.driveSystem.driveArcMpValues(1.3, 90, true, Constants.arcMPValue));
		addSequential(new ReleaseCube(), 1);
		
//		addSequential(Robot.driveSystem.driveArcMpValues(1, -95, true, Constants.arcMPValue));
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(3.2, false));
//		addParallel(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
//		addSequential(Robot.driveSystem.driveArcMpValues(1.2, 170, true, Constants.arcMPValue));
//		addSequential(new ReleaseCube(), 1);
		
	}

}
