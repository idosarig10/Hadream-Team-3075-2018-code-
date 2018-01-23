
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3075.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap 
{
	////chassis////
	
	public static final int rearLeft = 1;
    public static final int frontLeft = 2;
    
    public static final int rearRight = 9;
    public static final int frontRight = 8;
    
    public static final int shifterForward = 6;
    public static final int shifterReverse = 7;
    
    public static final int leftWheel = 7;
    public static final int rightWheel = 5;
    public static final int conveyor = 6;
    
    
    public static final int bigElevatorMasterMotor = 3;
    public static final int smallElevatorMotor = 4;
    public static final int bigElevatorSlaveMotor = 5;
    
    public static final int highMicroSwitch = 11;
    public static final int lowMicroSwitch = 12;
    public static final int middleMicroSwitch = 13;
}
