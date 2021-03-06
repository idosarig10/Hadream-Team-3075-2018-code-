package LibPurple.control;


public class GyroTrajectory extends TrajectoryTMP
{
	private double circ;
	private boolean clockwise;
	
	public GyroTrajectory(double distance, double maxA, double maxV, double radius, boolean clockwise)
	{
		super(distance, maxA, maxV);
		this.circ = 2 * Math.PI * radius;
		this.clockwise = clockwise;
	}
	
	public double getFinalAngle()
	{
		return 360 * this.getDistance() / this.circ;
	}
	
	public double getHeading()
	{
		if(clockwise)
			return this.circ == 0 ? 0 : 360 * this.setpoint.position / this.circ;
		else
			return this.circ == 0 ? 0 : -(360 * this.setpoint.position / this.circ);
	}
}