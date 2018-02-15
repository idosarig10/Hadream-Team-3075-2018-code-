package Autonomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;
import org.usfirst.frc.team3075.robot.commands.Wait;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftSwitch extends CommandGroup 
{

    public LeftSwitch() 
    {
        addSequential(Robot.driveSystem.driveStraightTrapizodial(3, false));
        addParallel(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
        addSequential(Robot.driveSystem.driveArcMpValues(0.4, 90, true, Constants.arcMPValue));
    	addSequential(new ReleaseCube(), 1);
    	
//    	addSequential(Robot.driveSystem.turnAngleTrapizodial(-90));
//    	addSequential(new AutoSetSmallElevator(Constants.smallElevatorDownPosition));
//    	addSequential(Robot.driveSystem.driveStraightTrapizodial(0.9, false));
    	
//    	addSequential(Robot.driveSystem.driveStraightTrapizodial(4, false));
//		addSequential(Robot.driveSystem.turnAngleTrapizodial(90),2.5);
//		addSequential(new Wait(0.1));
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.3, false));
//		addSequential(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
//		addSequential(new ReleaseCube(), 1);
//		addSequential(Robot.driveSystem.turnAngleTrapizodial(-90), 2.5);
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(1.4, false));
    }
}
