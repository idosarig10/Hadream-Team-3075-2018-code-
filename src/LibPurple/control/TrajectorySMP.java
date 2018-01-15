package LibPurple.control;

/**
 * 
 * @author 3075programming
 * A trajectory generator that uses sinusoidal functions
 * inspired by Ether's white paper
 * https://www.chiefdelphi.com/media/papers/3107
 */
public class TrajectorySMP extends Trajectory3075 
{
	
	private double T;
	private double K;
	private double distance;
	private double direction;
	
	public TrajectorySMP(double distance, double maxA)
	{
		this.direction = Math.signum(distance);
		this.setpoint = new Setpoint();
		
		this.distance = distance;
		this.maxA = maxA;
		
		T = Math.sqrt((Math.abs(distance * Math.PI * 2)) / maxA);
		K = (2 * Math.PI) / T;
	}
	
	@Override
	public Trajectory3075.Setpoint calculate(double time)
	{	
		setpoint.position = (-(maxA / (K * K)) * Math.sin(K * time) + maxA/K * time) * this.direction;
		setpoint.velocity = (-(maxA / K) * Math.cos(K * time) + maxA/K) * this.direction;
		setpoint.acceleration = (maxA * Math.sin(K * time)) * this.direction;
		return setpoint;
	}

	@Override
	public double getTotalTime() 
	{
		return T;
	}

	@Override
	public double getDistance() 
	{
		return this.distance;
	}

	@Override
	public void setDistance(double distance) 
	{
		this.distance = distance;
	}
}
