package Autonomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftScale extends CommandGroup
{

    public LeftScale() 
    {

    	addSequential(Robot.driveSystem.driveStraightRelativeTolerance(8, 0.01));
		addSequential(Robot.driveSystem.turnAngleTrapizodial(-90));
		addSequential(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
		addSequential(new ReleaseCube(), 1);
    }
}
