package Autonomous;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;
import org.usfirst.frc.team3075.robot.commands.Wait;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousLeftScale extends CommandGroup
{
	public AutonomousLeftScale()
	{
		if(Robot.gameData.length() > 0)
		{
			if (Robot.gameData.charAt(1) == 'L')
			{
				
				addSequential(new LeftScale());
			}
			else if(Robot.gameData.charAt(0) == 'L')
			{
				
				addSequential(new LeftSwitch());
			}
			else
			{
				
			}
		}
	}
}
