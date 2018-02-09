package Atunomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;

import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousLeftSwitch extends CommandGroup{
	public AutonomousLeftSwitch()
	{
		addSequential(Robot.driveSystem.driveStraightTrapizodial(4, false));
//		Utils.print("Finished Moving Straight");
		addSequential(Robot.driveSystem.turnAngleTrapizodial(85));
		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.2, false));
		addSequential(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
		addSequential(new ReleaseCube());
	}
}