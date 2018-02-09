package LibPurple.control;

public class GyroSetpoint extends Trajectory3075.Setpoint
{
	public double heading;
	
	public GyroSetpoint(double position, double velocity, double acceleration, double heading)
	{
		super(position, velocity, acceleration);
		this.heading = heading;
	}
	
	public GyroSetpoint()
	{
		this.velocity = 0;
		this.position = 0;
		this.acceleration = 0;
		this.heading = 0;
	}
	
}
