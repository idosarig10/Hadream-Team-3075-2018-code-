package org.usfirst.frc.team3075.robot;

import LibPurple.control.PIDvalue;
import LibPurple.control.MPController.MPValue;

public class Constants
{
	////chassis/////
	public static final double rightDistancePerPulse = 17000;
	public static final double leftDistancePerPulse = 17000;
	public static final double distancePerAngle = 110.5;
	
	public static final double powerRightMaxV = 2.04;
	public static final double powerLeftMaxV = 2.1;
	
	public static final double rightMaxA = 1.5;
	public static final double leftMaxA = 1.5;
	
	public static final double rightTurnMaxV = 0;
	public static final double leftTurnMaxV = 0;
	
	public static final double rightTurnMaxA = 0;
	public static final double leftTurnMaxA = 0;
	
	public static final PIDvalue rightVelocityPID = new PIDvalue(0, 0, 0, 0);
	public static final PIDvalue leftVelocityPID = new PIDvalue(0, 0, 0, 0);

	public static final PIDvalue rightTurnPID = new PIDvalue(0, 0, 0, 0);
	public static final PIDvalue leftTurnPID = new PIDvalue(0, 0, 0 , 0);

	public static final PIDvalue rightPositionPID = new PIDvalue(.08, 0.4, 0);
	public static final PIDvalue leftPositionPID = new PIDvalue(.15, 0.4, 0);

	public static final MPValue rightMPValue = new MPValue(rightPositionPID, 1 / powerRightMaxV, 0.05);
	public static final MPValue leftMPValue = new MPValue(leftPositionPID, 1/powerLeftMaxV, 0.05);

	public static final MPValue rightTurnMP = new MPValue(rightTurnPID, 0, 0);
	public static final MPValue leftTurnMP = new MPValue(leftTurnPID, 0, 0);
	
	public static final double turnAngleTolerance = 0;

	public static final double positionTolerance = 0.03;
	
	public static final double robotWidth = 0.66;
}