package org.usfirst.frc.team3075.robot.subsystems;

import org.usfirst.frc.team3075.robot.Constants;
import org.usfirst.frc.team3075.robot.Robot;
import org.usfirst.frc.team3075.robot.RobotMap;
import org.usfirst.frc.team3075.robot.subsystems.Chassis.Shift;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import LibPurple.actuator.Solenoid3075;
import LibPurple.sensors.Encoder3075;
import LibPurple.sensors.EncoderTalon3075;
import LibPurple.systems.DriveSystem3075;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Chassis extends DriveSystem3075 
{

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private WPI_TalonSRX frontLeft;
	private WPI_TalonSRX frontRight;

	private WPI_TalonSRX rearLeft;
	private WPI_TalonSRX rearRight;

	private Solenoid3075 shifter;

<<<<<<< HEAD
	private Encoder3075 leftEncoder;
	private Encoder3075 rightEncoder;
=======
	private EncoderTalon3075 leftEncoder;
	private EncoderTalon3075 rightEncoder;
>>>>>>> latesttryupdate

	public static enum Shift
	{
		Speed, Power
	}
	
	public Chassis()
	{
		frontLeft = new WPI_TalonSRX(RobotMap.frontLeft);
		frontRight = new WPI_TalonSRX(RobotMap.frontRight);
		rearLeft = new WPI_TalonSRX(RobotMap.rearLeft);
		rearRight = new WPI_TalonSRX(RobotMap.rearRight);

		frontRight.setInverted(true);
<<<<<<< HEAD
=======
		rearRight.setInverted(true);
>>>>>>> latesttryupdate

		shifter = new Solenoid3075(RobotMap.shifterForward, RobotMap.shifterReverse);
		leftEncoder = new EncoderTalon3075(frontLeft);
		rightEncoder = new EncoderTalon3075(frontRight);
		frontLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
		frontRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);

		super.initialize(frontRight, frontLeft, rightEncoder, leftEncoder);

		rearLeft.set(ControlMode.Follower, frontLeft.getDeviceID());
		rearRight.set(ControlMode.Follower, frontRight.getDeviceID());

		rightEncoder.setDistancePerPulse(Constants.distancePerPulse);
		leftEncoder.setDistancePerPulse(Constants.distancePerPulse);
		super.distancePerAngle = Constants.distancePerAngle;
<<<<<<< HEAD

		leftEncoder.setReverseDirection(true);
=======
  
		leftEncoder.setReverseDirection(true);
		rightEncoder.setReverseDirection(true);
>>>>>>> latesttryupdate

		super.rightMaxV = Constants.powerRightMaxV;
		super.leftMaxV = Constants.powerLeftMaxV;
		super.rightMaxA = Constants.rightMaxA;
		super.leftMaxA = Constants.leftMaxA;

		super.rightTurnMaxV = Constants.rightTurnMaxV;
		super.leftTurnMaxV = Constants.leftTurnMaxV;
		super.rightTurnMaxA = Constants.rightTurnMaxA;
		super.leftTurnMaxA = Constants.leftTurnMaxA;

		super.setPIDValues(Constants.leftVelocityPID, Constants.rightVelocityPID);
		super.setMPValues(Constants.leftMPValue, Constants.rightMPValue);
		super.setTurnMPValues(Constants.leftTurnMP, Constants.rightTurnMP);
		super.setVelocityTolerance(0);
		super.positionTolerance = Constants.positionTolerance;
		super.angleTolerance = Constants.turnAngleTolerance;

		super.robotWidth = Constants.robotWidth;
	}


	public void initDefaultCommand() 
	{
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
<<<<<<< HEAD
		setDefaultCommand(super.xboxArcadeDrive(Robot.m_oi.xbox));
=======
		setDefaultCommand(super.xboxArcadeDrive(Robot.oi.xbox));
>>>>>>> latesttryupdate
	}
	
	public Shift getShift()
	{
		return (shifter.get() == Value.kForward) ? Shift.Power : Shift.Speed;
	}
	
	public void setShifter(Shift shift)
	{
		switch(shift)
		{
		case Power:
			shifter.set(Value.kForward);
//			frontRight.enableBrakeMode(false);
//			rearRight.enableBrakeMode(true);
//			frontLeft.enableBrakeMode(false);
//			rearLeft.enableBrakeMode(true);
			
			super.rightMaxV = Constants.powerRightMaxV;
			super.leftMaxV = Constants.powerLeftMaxV;

			Constants.rightVelocityPID.kF = 1 / Constants.powerRightMaxV;
			Constants.leftVelocityPID.kF = 1 / Constants.powerRightMaxV;
			
			break;
			
		case Speed:
			shifter.set(Value.kReverse);
//			frontRight.enableBrakeMode(true);
//			rearRight.enableBrakeMode(true);
//			frontLeft.enableBrakeMode(true);
//			rearLeft.enableBrakeMode(true);
			
//			super.rightMaxV = Constants.speedShiftRightMaxV;
//			super.leftMaxV = Constants.speedShiftLeftMaxV;

//			Constants.rightVelocityPID.kF = 1 / Constants.speedShiftRightMaxV;
//			Constants.leftVelocityPID.kF = 1 / Constants.speedShiftLeftMaxV;
	        
			break;
			
		default: shifter.set(shifter.get());
		}
	}

	
<<<<<<< HEAD
	public Encoder3075 getLeftEncoder() {
=======
	public EncoderTalon3075 getLeftEncoder() {
>>>>>>> latesttryupdate
		return leftEncoder;
	}

	
<<<<<<< HEAD
	public Encoder3075 getRightEncoder() {
=======
	public EncoderTalon3075 getRightEncoder() {
>>>>>>> latesttryupdate
		return rightEncoder;
	}

	public WPI_TalonSRX getRearLeft() {
		return rearLeft;
	}

	public WPI_TalonSRX getFrontLeft() {
		return frontLeft;
	}

	public WPI_TalonSRX getRearRight() {
		return rearRight;
	}

	public WPI_TalonSRX getFrontRight() {
		return frontRight;
	}
}
