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
