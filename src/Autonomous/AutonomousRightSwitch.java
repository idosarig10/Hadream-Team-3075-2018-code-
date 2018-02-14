package Autonomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;
import org.usfirst.frc.team3075.robot.commands.Wait;

import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousRightSwitch extends CommandGroup{
	public AutonomousRightSwitch()
	{
		if(Robot.gameData.length() > 0)
		{
			if (Robot.gameData.charAt(0) == 'R')
			{
				addSequential(Robot.driveSystem.driveStraightTrapizodial(4, false));
				addSequential(Robot.driveSystem.turnAngleTrapizodial(-90),2.5);
				addSequential(new Wait(0.1));
				addSequential(Robot.driveSystem.driveStraightTrapizodial(0.3, false));
				addSequential(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
				addSequential(new ReleaseCube(), 1);
				addSequential(Robot.driveSystem.turnAngleTrapizodial(90), 2.5);
				addSequential(Robot.driveSystem.driveStraightTrapizodial(1.4, false));
			}
			else if(Robot.gameData.charAt(1) == 'R')
			{
				addSequential(Robot.driveSystem.driveStraightRelativeTolerance(8, 0.01));
				addSequential(Robot.driveSystem.turnAngleTrapizodial(90));
				addSequential(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
				addSequential(new ReleaseCube(), 1);
			}
			else
			{

			}
		}
	}
}