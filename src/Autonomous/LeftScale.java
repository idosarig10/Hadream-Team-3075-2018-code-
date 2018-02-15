package Autonomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.AutoSetElevators;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;
import org.usfirst.frc.team3075.robot.commands.Wait;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftScale extends CommandGroup
{

    public LeftScale() 
    {

    	addSequential(Robot.driveSystem.driveStraightTrapizodial(6.9, false), 5.8);
    	addSequential(new Wait(0.5));
		addSequential(Robot.driveSystem.turnAngleTrapizodial(30));
//		addSequential(new AutoSetElevators(Constants.bigElevatorTopPosition, Constants.smallElevatorTopPosition));
//		addSequential(new ReleaseCube(), 1);
    }
}
