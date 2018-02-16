package Autonomous;

import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.ActiveIntake;
import org.usfirst.frc.team3075.robot.commands.Wait;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MiddlePath extends CommandGroup {

    public MiddlePath() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.
//    	addSequential(Robot.driveSystem.drive2D("/Paths/middle_intake_left.csv", "/Paths/middle_intake_right.csv"), 3);
    	addSequential(new Wait(0.5));
    	addParallel(new ActiveIntake(0.7, 0.7), 3);
    	addParallel(Robot.driveSystem.driveStraightTrapizodial(0.3, 0.25, 0.5));
		addSequential(Robot.driveSystem.driveStraightTrapizodial(0.3, false));
    	
        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
