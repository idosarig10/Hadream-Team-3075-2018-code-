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
	
	private WPI_TalonSRX bigElevatorMasterMotor;
	private WPI_TalonSRX bigElevatorSlaveMotor;
	public WPI_TalonSRX smallElevatorMotor ;
	private Encoder bigElevatorEncoder;
	private Encoder smallElevatorEncoder;
	
	private static DigitalInput highMicroSwitch ;
	private static DigitalInput middleMicroSwitch ;
	
	private PIDController bigElevatorPID;
	public PIDController smallElevatorPID;
	public Solenoid3075 climbShifter;
	
	public Elevator()
	{
		bigElevatorMasterMotor = new WPI_TalonSRX(RobotMap.bigElevatorMasterMotor);
		bigElevatorSlaveMotor = new WPI_TalonSRX(RobotMap.bigElevatorSlaveMotor);
		smallElevatorMotor = new WPI_TalonSRX(RobotMap.smallElevatorMotor);
		
		bigElevatorEncoder = new Encoder(2, 3);
		bigElevatorPID = new PIDController(0, 0, 0, bigElevatorEncoder, bigElevatorMasterMotor);
		bigElevatorPID.setAbsoluteTolerance(Constants.bigElevatorTolerance);
		smallElevatorEncoder = new Encoder(0, 1);
		
		smallElevatorPID = new PIDController(0.000006, 0.0000001, 0.00009, smallElevatorEncoder, smallElevatorMotor);
		smallElevatorPID.setAbsoluteTolerance(Constants.smallElevatorTolerance);
		
		bigElevatorMasterMotor.setInverted(true);
		bigElevatorSlaveMotor.setInverted(true);
		smallElevatorMotor.setInverted(false);
		
		smallElevatorEncoder.setReverseDirection(true);
		
		highMicroSwitch = new DigitalInput(RobotMap.highMicroSwitch);
		middleMicroSwitch = new DigitalInput(RobotMap.middleMicroSwitch);
		bigElevatorSlaveMotor.set(ControlMode.Follower, bigElevatorMasterMotor.getDeviceID());
		climbShifter = new Solenoid3075(RobotMap.climbSolenoidForward, RobotMap.climbSolenoidBackward);
	}
	
	public void resetEncoders()
	{
		this.bigElevatorEncoder.reset();
		this.smallElevatorPID.setSetpoint(0);
		this.smallElevatorEncoder.reset();
	}
	
	public void setBigElevatorMasterMotor(double speed) 
	{
		bigElevatorMasterMotor.set(speed);
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

