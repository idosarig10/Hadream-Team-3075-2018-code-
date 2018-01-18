<<<<<<< HEAD
//package LibPurple.control;
//
//import LibPurple.control.Trajectory3075.Setpoint;
//
//public class TrajectoryTMP extends Trajectory3075
//{
//	
////	private double T;
////	private double K;
//	private double distance;
//	private double direction;
//	
//	public TrajectoryTMP(double distance, double MAX_A)
//	{
//		this.direction = Math.signum(distance);
//		this.setpoint = new Setpoint();
//		
//		this.distance = distance;
//		this.maxA = MAX_A;
//		
////		T = Math.sqrt((Math.abs(distance * Math.PI * 2)) / MAX_A);
////		K = (2 * Math.PI) / T;
//	}
//	
//	@Override
//	public Trajectory3075.Setpoint calculate(double time)
//	{
//		double a = maxA;
//		double v = a * time;
//		double p = (time*v)/2
//		if(time >= )
//		return setpoint;
//	}
//
//	@Override
//	public double getTotalTime() 
//	{
//		return T;
//	}
//
//	@Override
//	public double getDistance() 
//	{
//		return this.distance;
//	}
//
//	@Override
//	public void setDistance(double distance) 
//	{
//		this.distance = distance;
//	}
//}
=======
package LibPurple.control;

import LibPurple.control.Trajectory3075.Setpoint;

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
>>>>>>> latesttryupdate
