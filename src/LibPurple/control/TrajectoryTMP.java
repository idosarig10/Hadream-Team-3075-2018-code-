package LibPurple.control;

import LibPurple.control.Trajectory3075.Setpoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryTMP extends Trajectory3075
{
	
	private double T;
	private double distance;
	private double direction;
	
	public TrajectoryTMP(double distance, double maxA, double maxV)
	{
		this.setpoint = new Setpoint();
		this.direction = Math.signum(distance);
		this.distance = distance;
		this.maxA = maxA * direction;
		this.maxV = maxV * direction;
		
		this.T = distance/this.maxV + this.maxV/this.maxA;
	}
	
	@Override
	public Trajectory3075.Setpoint calculate(double time)
	{
		T = maxV < ((T/2) * maxA) ? T : Math.sqrt((4 * this.distance) / this.maxA);
		maxV = Math.min(maxV, (this.T / 2) * this.maxA);
		double b = T - ((2 * maxV) / maxA); // the upper base of the trapezoid
		double x = maxV / maxA; // the time it takes to accelerate to maxV
		
		if(time < x)
		{
			double a = maxA;
			double v = a * time;
			double p = (time * v) / 2;
			setpoint.acceleration = a;
			setpoint.velocity = v;
			setpoint.position = p;
		}
		else if(time <= T - x)
		{
			double a = 0;
			double v = maxV;
			double triangleArea = (x * maxV) / 2;
			double p = ((time - x) * maxV) + triangleArea;
			setpoint.acceleration = a;
			setpoint.velocity = v;
			setpoint.position = p;
		}
		else if(time > T - x)
		{
			double a = -maxA;
			double v = -maxA * (time - T);
			double triangleArea = (x * maxV) / 2;
			double rectangleArea = maxV  * b;
			double p = (((time - b - x) * (maxV + v)) / 2) + triangleArea + rectangleArea;
			setpoint.acceleration = a;
			setpoint.velocity = v;
			setpoint.position = p;
		}
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
	
	public double getDirection() {
		return direction;
	}

}
