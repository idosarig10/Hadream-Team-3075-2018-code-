package org.usfirst.frc.team3075.robot;

import LibPurple.control.PIDvalue;
import LibPurple.control.GyroMPController.GyroMPValue;
import LibPurple.control.MPController.MPValue;

////////Robot B////////
public class Constants
{
	////chassis/////
	
//	public static final double rightDistancePerPulse = 7269;
//	public static final double leftDistancePerPulse = 7216;
	
	public static final double rightDistancePerPulse = 7524;
	public static final double leftDistancePerPulse = 7511;
	public static final double distancePerAngle = 0.0061;
	
	public static final double powerRightMaxV = 1.6;
	public static final double powerLeftMaxV = 1.6;
	
	public static final double rightMaxA = 1.25;
	public static final double leftMaxA = 1.25;
	
	public static final double rightTurnMaxV = 1.8493;
	public static final double leftTurnMaxV = 1.8082;
	
	public static final double rightTurnMaxA = 1.5;
	public static final double leftTurnMaxA = 1.5;
	
	public static final double KGyro = 0.3;
	
	public static final PIDvalue rightVelocityPID = new PIDvalue(0, 0, 0, 0);
	public static final PIDvalue leftVelocityPID = new PIDvalue(0, 0, 0, 0);

	public static final PIDvalue rightTurnPID = new PIDvalue(6, 2, 0.05);
	public static final PIDvalue leftTurnPID = new PIDvalue(6, 2, 0.05);

//	public static final PIDvalue rightPositionPID = new PIDvalue(7.5, 1, 0.1);
//	public static final PIDvalue leftPositionPID = new PIDvalue(7.5, 1, 0.1);
	public static final PIDvalue rightPositionPID = new PIDvalue(4, 1, 0.05);
	public static final PIDvalue leftPositionPID = new PIDvalue(4, 1, 0.05);
	
	public static final PIDvalue rightGyroPID = new PIDvalue(0, 0, 0);
	public static final PIDvalue leftGyroPID = new PIDvalue(0, 0, 0);
	
	public static final PIDvalue arcPositionPID = new PIDvalue(4, 2, 0.1);
	
	public static final GyroMPValue arcMPValue = new GyroMPValue(arcPositionPID, 1 / powerRightMaxV, 0.1, KGyro);
	
	public static final MPValue rightMPValue = new MPValue(rightPositionPID, 1 / powerRightMaxV, 0.1);
	public static final MPValue leftMPValue = new MPValue(leftPositionPID, 1 / powerLeftMaxV, 0.1);
	
	public static final GyroMPValue rightGyroMPValue = new GyroMPValue(rightPositionPID, powerRightMaxV, 0.1, KGyro);
	public static final GyroMPValue leftGyroMPValue = new GyroMPValue(leftPositionPID, 1 /powerLeftMaxV, 0.1, KGyro);

	public static final MPValue rightTurnMP = new MPValue(rightTurnPID, 1 / rightTurnMaxV, 0.1);
	public static final MPValue leftTurnMP = new MPValue(leftTurnPID, 1 / leftTurnMaxV, 0.1);
	
	public static final double turnAngleTolerance = 1.5;

	public static final double positionTolerance = 0.05;
	
	public static final double robotWidth = 0.66;

	public static final double maxPositionValue = 0;
	
	////elevator////
	
	public static final double bigElevatorTolerance = 0;
	public static final double smallElevatorTolerance = 5000;
	
	public static final double smallElevatorTopPosition = 300000;
	public static final double smallElevatorDownPosition = 5000;
	
	public static final double bigElevatorTopPosition = -6800;
	public static final double bigelevatorDownPosition = 0;
}


///////Robot A///////
//public class Constants
//{
//	////chassis/////
//	
//	public static final double rightDistancePerPulse = 6740;
//	public static final double leftDistancePerPulse = 6740;
//	public static final double distancePerAngle = 0;
//	
//	public static final double powerRightMaxV = 1.8;
//	public static final double powerLeftMaxV = 1.8;
//	
//	public static final double rightMaxA = 1;
//	public static final double leftMaxA = 1;
//	
//	public static final double rightTurnMaxV = 0;
//	public static final double leftTurnMaxV = 0;
//	
//	public static final double rightTurnMaxA = 0;
//	public static final double leftTurnMaxA = 0;
//	
//	public static final double KGyro = 0.005555;
//	
//	public static final PIDvalue rightVelocityPID = new PIDvalue(0, 0, 0, 0);
//	public static final PIDvalue leftVelocityPID = new PIDvalue(0, 0, 0, 0);
//
//	public static final PIDvalue rightTurnPID = new PIDvalue(0, 0, 0);
//	public static final PIDvalue leftTurnPID = new PIDvalue(0, 0, 0);
//
//	public static final PIDvalue rightPositionPID = new PIDvalue(0, 0, 0);
//	public static final PIDvalue leftPositionPID = new PIDvalue(0, 0, 0);
//	
//	public static final PIDvalue rightGyroPID = new PIDvalue(0, 0, 0);
//	public static final PIDvalue leftGyroPID = new PIDvalue(0, 0, 0);
//	
//	public static final PIDvalue arcPositionPID = new PIDvalue(0, 0, 0);
//	
//	public static final MPValue arcMPValue = new MPValue(arcPositionPID, 1 / powerRightMaxV, 0.3);
//	
//	public static final MPValue rightMPValue = new MPValue(rightPositionPID, 1 / powerRightMaxV, 0);
//	public static final MPValue leftMPValue = new MPValue(leftPositionPID, 1/powerLeftMaxV, 0.1);
//	
//	public static final GyroMPValue rightGyroMPValue = new GyroMPValue(rightGyroPID, 0, 0, KGyro);
//	public static final GyroMPValue leftGyroMPValue = new GyroMPValue(leftGyroPID, 0, 0, KGyro);
//
//	public static final MPValue rightTurnMP = new MPValue(rightTurnPID, 1 / rightTurnMaxV, 0);
//	public static final MPValue leftTurnMP = new MPValue(leftTurnPID, 1 / leftTurnMaxV, 0);
//	
//	public static final double turnAngleTolerance = 1.5;
//
//	public static final double positionTolerance = 0.02;
//	
//	public static final double robotWidth = 0.67;
//
//	public static final double maxPositionValue = 0;
//	
//	////elevator////
//	
//	public static final double bigElevatorTolerance = 0;
//	public static final double smallElevatorTolerance = 0;
//	
//	public static final double smallElevatorTopPosition = 0;
//	public static final double smallElevatorDownPosition = 0;
//}
