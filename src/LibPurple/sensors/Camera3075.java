package LibPurple.sensors;

import org.usfirst.frc.team3075.robot.Robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class Camera3075 implements PIDSource{

	private PIDSourceType pidSourceType;
	
	public Camera3075()
	{
		this.pidSourceType = PIDSourceType.kDisplacement;
	}
	
	@Override  
	public void setPIDSourceType(PIDSourceType pidSource)
	{
		this.pidSourceType = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return pidSourceType;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return Robot.server.getCenter().x;
	}

}
