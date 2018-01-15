package org.usfirst.frc.team3075.robot;

import LibPurple.control.PIDvalue;
import LibPurple.control.MPController.MPValue;

public class Constants
{
	////chassis/////
	public static final double distancePerPulse = 16791;
	public static final double distancePerAngle = 110.5;
	
	public static final double powerRightMaxV = 2;
	public static final double powerLeftMaxV = 2;
	
	public static final double rightMaxA = 4;
	public static final double leftMaxA = 4;
	
	public static final double rightTurnMaxV = 0;
	public static final double leftTurnMaxV = 0;
	
	public static final double rightTurnMaxA = 0;
	public static final double leftTurnMaxA = 0;
	
	public static final PIDvalue rightVelocityPID = new PIDvalue(0, 0, 0, 0);
	public static final PIDvalue leftVelocityPID = new PIDvalue(0, 0, 0, 0);

	public static final PIDvalue rightPositionPID = new PIDvalue(0.05, 0.001, 0);
	public static final PIDvalue leftPositionPID = new PIDvalue(0.05, 0.001, 0);

	public static final PIDvalue rightTurnPID = new PIDvalue(0, 0, 0, 0);
	public static final PIDvalue leftTurnPID = new PIDvalue(0, 0, 0, 0);

	public static final MPValue rightMPValue = new MPValue(rightPositionPID, 1 / powerLeftMaxV, 0.0666);
	public static final MPValue leftMPValue = new MPValue(leftPositionPID, 1/powerLeftMaxV, 0.0666);

	public static final MPValue rightTurnMP = new MPValue(rightTurnPID, 0, 0);
	public static final MPValue leftTurnMP = new MPValue(leftTurnPID, 0, 0);
	
	public static final double positionTolerance = 20;
	public static final double turnAngleTolerance = 0;
	
	public static final double robotWidth = 0.68;
}