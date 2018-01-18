package org.usfirst.frc.team3075.robot.subsystems;

import org.usfirst.frc.team3075.robot.RobotMap;
//import org.usfirst.frc.team3075.robot.commands.ElevatorCotrol;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
//public class Elevator extends Subsystem 
//{
//
//    // Put methods for controlling this subsystem
//    // here. Call these from Commands.
//	
//	private static WPI_TalonSRX bigElevatorMasterMotor;
//	private static WPI_TalonSRX bigElevatorSlaveMotor;
//	private static WPI_TalonSRX smallElevatorMotor ;
//
//	
//	private static DigitalInput highMicroSwitch ;
//	private static DigitalInput middleMicroSwitch ;
//	
//
//	
//	public Elevator()
//	{
//		bigElevatorMasterMotor = new WPI_TalonSRX(RobotMap.bigElevatorMasterMotor);
//		bigElevatorSlaveMotor = new WPI_TalonSRX(RobotMap.bigElevatorSlaveMotor);
//		smallElevatorMotor = new WPI_TalonSRX(RobotMap.smallElevatorMotor);
//		
//		highMicroSwitch = new DigitalInput(RobotMap.highMicroSwitch);
//		middleMicroSwitch = new DigitalInput(RobotMap.middleMicroSwitch);
//	}
//	public void setBigElevatorMasterMotor(double speed) 
//	{
//		bigElevatorMasterMotor.set(speed);
//	}
//	public void setBigElevatorSlaveMotor(double speed)
//	{
//		bigElevatorSlaveMotor.set(speed);
//	}
//	public void setSmallElevatorMotor(double speed)
//	{
//		smallElevatorMotor.set(speed);
//	}
//    public void initDefaultCommand() 
//    {
//    	
//        // Set the default command for a subsystem here.
//        setDefaultCommand(new ElevatorCotrol());
//    }
//}
//
