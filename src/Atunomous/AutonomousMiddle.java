package Atunomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousMiddle extends CommandGroup
{
	public AutonomousMiddle()
	{
		addSequential(new AutonomousMiddlePickUp());
		addSequential(new AutonomousMiddleSwitchLeft());
	}

}
