package Autonomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.ActiveIntake;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;
import org.usfirst.frc.team3075.robot.commands.Wait;

//import com.sun.xml.internal.bind.v2.runtime.reflect.opt.Const;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MiddleSwitchLeft extends CommandGroup
{
	public MiddleSwitchLeft() 
	{
		// -------- option 1, 14.5 seconds -----
//		addSequential(new Wait(0.05));
//		addParallel(new ActiveIntake(0.5, 0.5));
//		addSequential(Robot.driveSystem.driveArcMpValues(0.50, -220, false, Constants.arcMPValue), 3.5);
//		addSequential(new Wait(0.05));
//		addParallel(new AutoSetSmallElevator(Constants.smallElevatorTopPosition), 3.5);
//		addParallel(new ActiveIntake(0.2, 0.2), 2);
//		addSequential(Robot.driveSystem.driveArcMpValues(1, 220, true, Constants.arcMPValue), 4.6);
//		addSequential(new ReleaseCube(), 1);
		
		addSequential(new Wait(0.05));
		addSequential(Robot.driveSystem.driveArc(0.4, -90, true, Constants.arcMPValue));
//		addSequential(Robot.driveSystem.driveArc(1.8, 60, true, Constants.arcMPValue));
//		addSequential(new Wait(0.05));
//		addSequential(Robot.driveSystem.driveArc(1, -200, false, Constants.arcMPValue));
//		addSequential(Robot.driveSystem.turnAngleTrapizodial(-90));
//		addParallel(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
//		addSequential(Robot.driveSystem.driveArcMpValues(1.3, 90, true, Constants.arcMPValue));
//		addSequential(new ReleaseCube(), 1);
		
//		addSequential(Robot.driveSystem.driveArcMpValues(1, -95, true, Constants.arcMPValue));
//		addSequential(Robot.driveSystem.driveStraightTrapizodial(3.2, false));
//		addParallel(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
//		addSequential(Robot.driveSystem.driveArcMpValues(1.2, 170, true, Constants.arcMPValue));
//		addSequential(new ReleaseCube(), 1);
		
	}

}
