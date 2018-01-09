package LibPurple.control;

import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class PIDCameraTurn implements PIDOutput{

	@Override
	public void pidWrite(double output) 
	{
		Robot.driveSystem.set(output, -output);
	}
	
}
