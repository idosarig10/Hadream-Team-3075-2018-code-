package LibPurple.control;


public class GyroTrajectory extends Trajectory3075
{
	private double T;
	private double distance;
	private double direction;
	private GyroSetpoint setpoint;
	private double scope;
	
	public GyroTrajectory(double distance, double maxA, double maxV, double radius)
	{
		this.setpoint = new GyroSetpoint();
		this.direction = Math.signum(distance);
		this.distance = distance;
		this.maxA = maxA * direction;
		this.maxV = maxV * direction;
		this.scope = 2 * Math.PI * radius;
		
		this.T = distance/this.maxV + this.maxV/this.maxA;
	}
	
	@Override
	public GyroSetpoint calculate(double time) 
	{
		if(this.direction > 0)
		{
			if(maxV > (T/2) * maxA)
				T = Math.sqrt((4 * this.distance) / this.maxA);
			maxV = Math.min(maxV, (this.T / 2) * this.maxA);
		}
		else
		{
			if(maxV < (T/2) * maxA)
				T = Math.sqrt((4 * this.distance) / this.maxA);
			maxV = Math.max(maxV, (this.T / 2) * this.maxA);
		}
		
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
		setpoint.heading = this.scope == 0 ? 0 : 360 * this.setpoint.position / this.scope;
		return setpoint;
	}

	@Override
	public double getTotalTime() {
		// TODO Auto-generated method stub
		return this.T;
	}

	@Override
	public double getDirection() {
		// TODO Auto-generated method stub
		return this.direction;
	}

	@Override
	public double getDistance() {
		// TODO Auto-generated method stub
		return this.distance;
	}
	
	public double getFinalAngle()
	{
		return this.scope == 0 ? 0 : 360 * this.distance / this.scope;
	}

	@Override
	public void setDistance(double distance) {
		// TODO Auto-generated method stub
		
	}	
}