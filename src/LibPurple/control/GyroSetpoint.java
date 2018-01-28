package LibPurple.control;

public class GyroSetpoint extends Trajectory3075.Setpoint
{
	public double velocity;
	public double position;
	public double acceleration;
	public double heading;
	
	public GyroSetpoint(double position, double velocity, double acceleration, double heading)
	{
		this.velocity = velocity;
		this.position = position;
		this.acceleration = acceleration;
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
