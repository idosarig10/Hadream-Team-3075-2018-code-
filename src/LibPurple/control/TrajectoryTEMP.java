package LibPurple.control;

import LibPurple.control.Trajectory3075.Setpoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryTEMP extends Trajectory3075
{

	private double T;
	private double distance;
	private double endV;
	private double x1;
	private double x2;
	private double b;
	private double direction;

	public TrajectoryTEMP(double distance, double maxA, double maxV, double endV)
	{
		this.setpoint = new Setpoint();
		this.direction = Math.signum(distance);
		this.distance = distance;
		this.maxA = maxA * direction;
		this.maxV = maxV * direction;
		this.endV = endV * direction;


		this.x1 = (this.maxV - this.endV) / 2;
		this.x2 = this.endV / this.maxA;
		this.b = (this.distance - ((this.endV*x2)/2) - x1*this.maxV - x1*this.endV) / 2;
		this.T = x2 + 2*x1 + b;
	}

	@Override
	public Trajectory3075.Setpoint calculate(double time)
	{
		if(time < (this.x1 + this.x2))
		{
			double a = maxA;
			double v = a * time;
			double p = (time * v) / 2;
			setpoint.acceleration = a;
			setpoint.velocity = v;
			setpoint.position = p;
		}
		else if(time <= b + (x1 + x2))
		{
			double a = 0;
			double v = maxV;
			double triangleArea = ((x1 + x2) * maxV) / 2;
			double p = ((time - (x1 + x2)) * maxV) + triangleArea;
			setpoint.acceleration = a;
			setpoint.velocity = v;
			setpoint.position = p;
		}
		else if(time > b + (x1 + x2))
		{
			double a = -maxA;
			double v = -maxA * (time - T);
			double triangleArea = ((x1 + x2) * maxV) / 2;
			double rectangleArea = maxV  * b;
			double p = (((time - b - (x1 + x2)) * (maxV + v)) / 2) + triangleArea + rectangleArea;
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