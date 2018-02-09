package org.usfirst.frc.team3075.robot.commands;

import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LockWheels extends CommandGroup {

    public LockWheels() {
        addSequential(Robot.driveSystem.driveStraightEndless(0));
    }
}
