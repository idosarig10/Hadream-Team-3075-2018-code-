package org.usfirst.frc.team3075.robot.subsystems;

import org.usfirst.frc.team3075.robot.RobotMap;
import org.usfirst.frc.team3075.robot.commands.ActiveIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import LibPurple.actuator.Servo3075;
import LibPurple.actuator.Solenoid3075;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem 
{
	
	private WPI_TalonSRX rightWheel;
	private WPI_TalonSRX leftWheel;
	private WPI_TalonSRX conveyor;
	
	public Servo3075 leftServo, rightServo;
	
	public Solenoid3075 wheelsCylinder;
	public boolean isFolded = true;
	
	public Intake()
	{
		rightWheel = new WPI_TalonSRX(RobotMap.leftWheel);
		leftWheel = new WPI_TalonSRX(RobotMap.rightWheel);
		conveyor = new WPI_TalonSRX(RobotMap.conveyor);
		wheelsCylinder = new Solenoid3075(6, 7);
		this.rightServo = new Servo3075(1);
		this.leftServo = new Servo3075(0);
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
		conveyor.set(speed);
	}
	
	public void setRightWheelSpeed(double speed)
	{
		rightWheel.set(speed);
	}
	
	public void setLeftWheelSpeed(double speed)
	{
		leftWheel.set(speed);
	}
	
	public void setSolenoid(Value value)
	{
		this.wheelsCylinder.set(value);
	}
}