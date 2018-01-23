package org.usfirst.frc.team3075.robot.subsystems;

import org.usfirst.frc.team3075.robot.RobotMap;
import org.usfirst.frc.team3075.robot.commands.ActiveIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem 
{
	
	public WPI_TalonSRX rightWheel;
	public WPI_TalonSRX leftWheel;
	public WPI_TalonSRX Conveyor;
	
	public Intake()
	{
		rightWheel = new WPI_TalonSRX(RobotMap.leftWheel);
		leftWheel = new WPI_TalonSRX(RobotMap.rightWheel);
		Conveyor = new WPI_TalonSRX(RobotMap.conveyor);
	}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
    public void initDefaultCommand() 
    {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setConveyorSpeed(double speed)
    {
		rightWheel.set(speed);
	}
	
	public void setRightWheelSpeed(double speed)
	{
		rightWheel.set(speed);
	}
	
	public void setLeftWheelSpeed(double speed)
	{
		leftWheel.set(speed);
	}
}