package LibPurple.control;

import LibPurple.control.Trajectory3075.Setpoint;
import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryTSEMP extends Trajectory3075
{

	private double T;
	private double distance;
	private double startV;
	private double endV;
	private double endSeg;
	private double startSeg;
	private double b;
	private double direction;

	public TrajectoryTSEMP(double distance, double maxA, double maxV, double startV,double endV)
	{
		this.setpoint = new Setpoint();
		this.direction = Math.signum(distance);
		this.distance = distance;
		this.maxA = maxA * direction;
		this.maxV = maxV * direction;
		this.endV = endV * direction;
		this.startV = startV * direction;

		this.endSeg = (this.maxV - this.endV) / this.maxA;
		this.startSeg = (this.endV - this.startSeg) / this.maxA;
		this.b = ((2 * this.distance) - (this.startV * this.startSeg) - (2 * this.endSeg * this.maxV) - (2 * this.endSeg * this.endV)) / (2 * this.maxV);
		this.T = this.startSeg + 2 * this.endSeg + b;
	}

	@Override
	public Trajectory3075.Setpoint calculate(double time)
	{
		if(time < (this.endSeg + this.startSeg))
		{
			double a = maxA;
			double v = a * time + this.startSeg;
			double sTriangleArea = (time * v) / 2;
			double sRectArea = this.startV * time; 
			double p = sTriangleArea + sRectArea;
			setpoint.acceleration = a;
			setpoint.velocity = v;
			setpoint.position = p;
		}
		else if(time <= b + (this.endSeg + this.startSeg))
		{
			double a = 0;
			double v = maxV;
			double sTriangleArea = ((this.startSeg + this.endSeg) * maxV) / 2;
			double sRectArea = this.startSeg + this.endSeg * this.startV;
			double p = ((time - (this.startSeg + this.endSeg)) * maxV) + sTriangleArea + sRectArea;
			setpoint.acceleration = a;
			setpoint.velocity = v;
			setpoint.position = p;
		}
		else if(time > b + (this.startSeg + this.endSeg))
		{
			double a = -maxA;
			double v = -maxA * (time - T);
			double sTriangleArea = ((this.startSeg + this.endSeg) * maxV) / 2;
			double sRectArea = this.startSeg + this.endSeg * this.startV;
			double mRectangleArea = maxV  * b;
			double eTriangleArea = (((time - b - (this.startSeg + this.endSeg)) * (maxV - this.endSeg)) / 2);
			double eRectArea = (time - b - (this.startSeg + this.endSeg)) * this.endSeg;
			double p = sTriangleArea + sRectArea + mRectangleArea + eTriangleArea + eRectArea;
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