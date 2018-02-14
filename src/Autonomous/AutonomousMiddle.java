package Autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousMiddle extends CommandGroup
{
	public AutonomousMiddle()
	{
		addSequential(new MiddlePickUp());
		addSequential(new MiddleSwitchLeft());
	}

}
