
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
	
	public static final int rearLeft = 2;
    public static final int frontLeft = 1;
    
    public static final int rearRight = 11;
    public static final int frontRight = 3;
    
    public static final int shifterForward = 0;
    public static final int shifterReverse = 1;
    
    ////intake////
    public static final int leftWheel = 5;
    public static final int rightWheel = 6;
    public static final int conveyor = 9;
    public static final int solenoidForward = 2;
    public static final int solenoidBackward = 3;
    
    ////elevator////
    public static final int bigElevatorMasterMotor = 7;
    public static final int smallElevatorMotor = 10;
    public static final int bigElevatorSlaveMotor = 8;
    
    public static final int climbSolenoidForward = 4;
    public static final int climbSolenoidBackward = 5;
    
    public static final int highMicroSwitch = 11;
    public static final int lowMicroSwitch = 12;
    public static final int middleMicroSwitch = 13;
	public static final int encoder = 16;
	
}
