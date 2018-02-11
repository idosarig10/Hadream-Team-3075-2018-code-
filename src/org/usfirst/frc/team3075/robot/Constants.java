package org.usfirst.frc.team3075.robot;

import LibPurple.control.PIDvalue;
import LibPurple.control.MPController.MPValue;

public class Constants
{
	////chassis/////
	
	public static final double rightDistancePerPulse = 7269;
	public static final double leftDistancePerPulse = 7216;
	public static final double distancePerAngle = 0.0061;
	
//	public static final double powerRightMaxV = 2.18;//tene's dick fell off after the history test! <3 <3 co co bo bo (: :p DO NOT DELETE! RASTA WAS HERE 9\11\01                                                                                                        9/2/18
//	public static final double powerLeftMaxV = 2.18;
	public static final double powerRightMaxV = 1.8;
	public static final double powerLeftMaxV = 1.8;
	
	public static final double rightMaxA = 1;
	public static final double leftMaxA = 1;
	
	public static final double rightTurnMaxV = 1.8493;
	public static final double leftTurnMaxV = 1.8082;
	
	public static final double rightTurnMaxA = 0.75;
	public static final double leftTurnMaxA = 0.75;
	
	public static final PIDvalue rightVelocityPID = new PIDvalue(0, 0, 0, 0);
	public static final PIDvalue leftVelocityPID = new PIDvalue(0, 0, 0, 0);

	public static final PIDvalue rightTurnPID = new PIDvalue(6, 2, 0.05);
	public static final PIDvalue leftTurnPID = new PIDvalue(6, 2, 0.05);

//	public static final PIDvalue rightPositionPID = new PIDvalue(7.5, 1, 0.1);
//	public static final PIDvalue leftPositionPID = new PIDvalue(7.5, 1, 0.1);

	public static final PIDvalue rightPositionPID = new PIDvalue(4, 1, 0.05);
	public static final PIDvalue leftPositionPID = new PIDvalue(4, 1, 0.05);

	public static final MPValue rightMPValue = new MPValue(rightPositionPID, 1 / powerRightMaxV, 0.1);
	public static final MPValue leftMPValue = new MPValue(leftPositionPID, 1/powerLeftMaxV, 0.1);

	public static final MPValue rightTurnMP = new MPValue(rightTurnPID, 1 / rightTurnMaxV, 0.1);
	public static final MPValue leftTurnMP = new MPValue(leftTurnPID, 1 / leftTurnMaxV, 0.1);
	
	public static final double turnAngleTolerance = 1.5;

	public static final double positionTolerance = 0.01;
	
	public static final double robotWidth = 0.66;

	public static final double maxPositionValue = 0;
	
	////elevator////
	
	public static final double bigElevatorTolerance = 0;
	public static final double smallElevatorTolerance = 5000;
	
	public static final double smallElevatorTopPosition = 300000;
	public static final double smallElevatorDownPosition = 5000;
}