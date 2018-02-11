package LibPurple.control;


public class GyroTrajectory extends TrajectoryTMP
{
	private double circ;
	
	public GyroTrajectory(double distance, double maxA, double maxV, double radius)
	{
		super(distance, maxA, maxV);
		this.circ = 2 * Math.PI * radius;
	}
	
	public double getFinalAngle()
	{
		return 360 * this.getDistance() / this.circ;
	}
	
	public double getHeading()
	{
		return this.circ == 0 ? 0 : 360 * this.setpoint.position / this.circ;
	}
}