package org.usfirst.frc.team3075.robot.subsystems;

import org.usfirst.frc.team3075.robot.RobotMap;
//import org.usfirst.frc.team3075.robot.commands.ElevatorCotrol;
import org.usfirst.frc.team3075.robot.commands.ManualElevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.usfirst.frc.team3075.robot.Constants;

import LibPurple.actuator.Solenoid3075;
import LibPurple.sensors.EncoderTalon3075;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem 
{

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public WPI_TalonSRX bigElevatorMotor;
	public WPI_TalonSRX smallElevatorMotor ;
	private Encoder bigElevatorEncoder;
	private Encoder smallElevatorEncoder;
	
	private static DigitalInput highMicroSwitch ;
	private static DigitalInput middleMicroSwitch ;
	
	public PIDController bigElevatorPID;
	public PIDController smallElevatorPID;
	
	public Elevator()
	{
		bigElevatorMotor = new WPI_TalonSRX(RobotMap.bigElevatorMasterMotor);
		smallElevatorMotor = new WPI_TalonSRX(RobotMap.smallElevatorMotor);
		
		bigElevatorEncoder = new Encoder(2, 3);
		bigElevatorPID = new PIDController(0.0007, 0.0000001, 0, -0.14, bigElevatorEncoder, bigElevatorMotor);
		bigElevatorPID.setAbsoluteTolerance(Constants.bigElevatorTolerance);
		smallElevatorEncoder = new Encoder(0, 1);
		
		smallElevatorPID = new PIDController(0.000003, 0.000000015, 0, smallElevatorEncoder, smallElevatorMotor);
		smallElevatorPID.setAbsoluteTolerance(Constants.smallElevatorTolerance);
		
		bigElevatorMotor.setInverted(true);
		smallElevatorMotor.setInverted(false);
		
		smallElevatorEncoder.setReverseDirection(true);
		
		highMicroSwitch = new DigitalInput(RobotMap.highMicroSwitch);
		middleMicroSwitch = new DigitalInput(RobotMap.middleMicroSwitch);
		
		bigElevatorPID.setOutputRange(-0.6, 0.6);
		smallElevatorPID.setOutputRange(-0.8, 0.8);
	}
	
	public void resetEncoders()
	{
		this.bigElevatorEncoder.reset();
		this.smallElevatorPID.setSetpoint(0);
		this.smallElevatorEncoder.reset();
		this.bigElevatorPID.setSetpoint(0);
	}
	
	public void setBigElevatorMasterMotor(double speed) 
	{
		bigElevatorMotor.set(speed);
	}
	
	public void setSmallElevatorMotor(double speed)
	{
		smallElevatorMotor.set(speed);
	}
	
    public void initDefaultCommand() 
    {	
        // Set the default command for a subsystem here.
        setDefaultCommand(new ManualElevator());
    }
    
	public double getPositionSmallEncoder() 
	{
		return this.smallElevatorEncoder.getDistance();
	}
	
	public double getPositionBigEncoder()
	{
		return this.bigElevatorEncoder.getDistance();
	}
	
	public double getPositionSwitch() 
	{
		return 0;
//		Can't find how to get the switch's positions.
	}
	
	public void activeBigElevatorPID()
	{
		bigElevatorPID.enable();
	}
	
	public void setBigElevatorSetpoint(double setpoint)
	{
		bigElevatorPID.setSetpoint(setpoint);
	}
	
	public void disableBigElevatorPID()
	{
		bigElevatorPID.disable();
	}
	
	public boolean bigElevatorOnTarget()
	{
		return bigElevatorPID.onTarget();
	}
	
	public void activeSmallElevatorPID()
	{
		smallElevatorPID.enable();
	}
	
	public void setSmallElevatorSetpoint(double setpoint)
	{
		smallElevatorPID.setSetpoint(setpoint);
	}
	
	public void disableSmallElevatorPID()
	{
		smallElevatorPID.disable();
	}
	
	public boolean smallElevatorOnTarget()
	{
		return smallElevatorPID.onTarget();
	}
}

