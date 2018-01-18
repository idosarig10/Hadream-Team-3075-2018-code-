package LibPurple.systems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import LibPurple.control.MPController;
import LibPurple.control.PIDvalue;
import LibPurple.control.TrajectorySMP;
import LibPurple.control.MPController.MPValue;
import LibPurple.sensors.ConsoleJoystick;
import LibPurple.sensors.Encoder3075;
import LibPurple.systems.DriveSystem3075.DrivingState;
import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class MagicDriveSystem3075 extends Subsystem implements Sendable
{
	public static enum DrivingState
	{
		Voltage, VelocityClosedLoop, DistanceMotionProfiled;
	}

	DrivingState state = DrivingState.Voltage;

	protected CANTalon rightMotor;
	protected CANTalon leftMotor;

	protected double rightMaxV;
	protected double leftMaxV;
	protected double rightMaxA;
	protected double leftMaxA;
	protected double rightTurnMaxV;
	protected double leftTurnMaxV;
	protected double rightTurnMaxA;
	protected double leftTurnMaxA;
	protected double distancePerAngle;
	protected double positionTolerance;
	protected double angleTolerance;
	protected double robotWidth;

	protected void initialize(CANTalon rightMotor, CANTalon leftMotor)
	{
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;

		this.rightMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		this.leftMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
	}

	@Override
	protected void initDefaultCommand() 
	{
		// TODO Auto-generated method stub	
	}

	public void set(double rightValue, double leftValue)
	{
		if(state == DrivingState.Voltage)
		{
			rightMotor.set(rightValue);
			leftMotor.set(leftValue);
		}
		//		else if(state == DrivingState.VelocityClosedLoop)
		//		{
		//			double maxV = Math.min(rightMaxV, leftMaxV);
		//			
		//			if(rightValue == 0)
		//			{
		//				rightPID.disable();
		//				rightPID.reset();
		//			}
		//			else
		//				rightPID.enable();	
		//			
		//			if(leftValue == 0)
		//			{
		//				leftPID.disable();
		//				leftPID.reset();
		//			}
		//			else
		//				leftPID.enable();
		//			
		//			rightPID.setSetpoint(rightValue * maxV);
		//			leftPID.setSetpoint(leftValue * maxV);
		//		}
	}

	public void enterState(DrivingState newState)
	{
		this.state = newState;

		switch(state)
		{
		case Voltage:
			break;
		case VelocityClosedLoop:
			//			rightEncoder.setPIDSourceType(PIDSourceType.kRate);
			//			leftEncoder.setPIDSourceType(PIDSourceType.kRate);
			//			setPIDEnabled(true);
			break;
		case DistanceMotionProfiled:
			rightMotor.changeControlMode(TalonControlMode.MotionMagic);
			leftMotor.changeControlMode(TalonControlMode.MotionMagic);
			break;
		default:
			break;

		}
	}

	private void setPIDEnabled(boolean enabled)
	{
		if(enabled)
		{
			//			rightPID.enable();
			//			leftPID.enable();
		}
		else
		{
			//			rightPID.disable();
			//			leftPID.disable();
		}
	}

	public void reset()
	{
		//		leftPID.reset();
		//		rightPID.reset();
	}

	//	public Command arcadeDrive(Joystick joystick)
	//	{
	//		return new ArcadeDriveCommand(this, joystick);
	//	}
	//
	//	public Command xboxArcadeDrive(ConsoleJoystick stick)
	//	{
	//		return new XboxArcade(this, stick);
	//	}
	//
	//	public Command tankDrive(Joystick rightJoystick, Joystick leftJoystick)
	//	{
	//		return new TankDriveCommand(this, rightJoystick, leftJoystick);
	//	}

	public Command driveStraight(double distance)
	{
		double maxA = Math.min(rightMaxA, leftMaxA);
		return new M_DriveDistance(this, distance, distance, false, maxA);
	}

	public Command driveStraightEndless(double distance)
	{
		double maxA = Math.min(rightMaxA, leftMaxA);
		return new M_DriveDistance(this, distance, distance, true, maxA);
	}

	//	public Command driveCurve(double radius, double angle, boolean clockwise)
	//	{
	//		double leftRadius = clockwise ? radius +  (robotWidth / 2) : radius - (robotWidth / 2);
	//		double rightRadius = clockwise ? radius -  (robotWidth / 2) : radius + (robotWidth / 2);
	//
	//		double leftDistance = Math.toRadians(angle) * leftRadius; 
	//		double rightDistance = Math.toRadians(angle) * rightRadius;
	//
	//		double leftMaxA = !clockwise ? 
	//				getMaxA() * (Math.min(leftRadius, rightRadius) / Math.max(leftRadius, rightRadius)) 
	//				: getMaxA(); 
	//				double rightMaxA = clockwise ? 
	//						getMaxA() * (Math.min(leftRadius, rightRadius) / Math.max(leftRadius, rightRadius)) 
	//						: getMaxA(); 
	//
	//
	//						return new DriveDistance(this, leftDistance, rightDistance, false, leftMaxA / 2, rightMaxA / 2);
	//	}

	//	public Command turnAngle(double angle)
	//	{			
	//		return new TurnAngle(this, angle,  false);
	//	}

	//	public Command turnAngleEndless(double angle)
	//	{
	//		return new TurnAngle(this, angle,  true);
	//	}

	public CANTalon getRightMotor() {
		return rightMotor;
	}

	public void setRightMotor(CANTalon rightMotor) {
		this.rightMotor = rightMotor;
	}

	public CANTalon getLeftMotor() {
		return leftMotor;
	}

	public void setLeftMotor(CANTalon leftMotor) {
		this.leftMotor = leftMotor;
	}

	public double getRightMaxV() {
		return rightMaxV;
	}

	public void setRightMaxV(double rightMaxV) {
		this.rightMaxV = rightMaxV;
	}

	public double getLeftMaxV() {
		return leftMaxV;
	}

	public void setLeftMaxV(double leftMaxV) {
		this.leftMaxV = leftMaxV;
	}

	public double getRightMaxA() {
		return rightMaxA;
	}

	public void setRightMaxA(double rightMaxA) {
		this.rightMaxA = rightMaxA;
	}

	public double getLeftMaxA() {
		return leftMaxA;
	}

	public void setLeftMaxA(double leftMaxA) {
		this.leftMaxA = leftMaxA;
	}

	public double getRightTurnMaxV() {
		return rightTurnMaxV;
	}

	public void setRightTurnMaxV(double rightTurnMaxV) {
		this.rightTurnMaxV = rightTurnMaxV;
	}

	public double getLeftTurnMaxV() {
		return leftTurnMaxV;
	}

	public void setLeftTurnMaxV(double leftTurnMaxV) {
		this.leftTurnMaxV = leftTurnMaxV;
	}

	public double getRightTurnMaxA() {
		return rightTurnMaxA;
	}

	public void setRightTurnMaxA(double rightTurnMaxA) {
		this.rightTurnMaxA = rightTurnMaxA;
	}

	public double getLeftTurnMaxA() {
		return leftTurnMaxA;
	}

	public void setLeftTurnMaxA(double leftTurnMaxA) {
		this.leftTurnMaxA = leftTurnMaxA;
	}

	public double getDistancePerAngle() {
		return distancePerAngle;
	}

	public void setDistancePerAngle(double distancePerAngle) {
		this.distancePerAngle = distancePerAngle;
	}

	public double getPositionTolerance() {
		return positionTolerance;
	}

	public void setPositionTolerance(double positionTolerance) {
		this.positionTolerance = positionTolerance;
	}

	public double getAngleTolerance() {
		return angleTolerance;
	}

	public void setAngleTolerance(double angleTolerance) {
		this.angleTolerance = angleTolerance;
	}

	public double getRobotWidth() {
		return robotWidth;
	}

	public void setRobotWidth(double robotWidth) {
		this.robotWidth = robotWidth;
	}

	public void setState(DrivingState state) {
		this.state = state;
	}

	//	public void setPIDValues(PIDvalue leftPIDValue, PIDvalue rightPIDValue) 
	//	{
	//		this.leftPIDValue = leftPIDValue;
	//		this.rightPIDValue = rightPIDValue;
	//
	//		if(this.leftPID != null)
	//			leftPID.setPID(leftPIDValue.kP, leftPIDValue.kI, leftPIDValue.kD, leftPIDValue.kF);
	//		else
	//			leftPID = new PIDController(leftPIDValue.kP, leftPIDValue.kI, leftPIDValue.kD, leftPIDValue.kF, leftEncoder, leftMotor);
	//
	//		if(this.rightPID != null)
	//			rightPID.setPID(rightPIDValue.kP, rightPIDValue.kI, rightPIDValue.kD, rightPIDValue.kF);
	//		else
	//			rightPID = new PIDController(rightPIDValue.kP, rightPIDValue.kI, rightPIDValue.kD, rightPIDValue.kF, rightEncoder, rightMotor);
	//	}

	public double getAngle()
	{
		return (leftMotor.getEncPosition() - rightMotor.getEncPosition()) / (2 * distancePerAngle);
	}

	//	public PIDvalue getLeftPIDValue() 
	//	{
	//		return leftPIDValue;
	//	}
	//
	//	public PIDvalue getRightPIDValue()
	//	{
	//		return rightPIDValue;
	//	}

	public DrivingState getState()
	{
		return state;
	}

	public void enterState(LibPurple.systems.DriveSystem3075.DrivingState distancemotionprofiled) {
		// TODO Auto-generated method stub

	}
}


class M_XboxArcade extends Command
{

	DriveSystem3075 driveSystem;
	ConsoleJoystick stick;

	private double leftValue;
	private double rightValue;
	private double last = 0;

	public M_XboxArcade(DriveSystem3075 driveSystem, ConsoleJoystick stick) {
		requires(driveSystem);
		this.driveSystem = driveSystem;
		this.stick = stick;
	}

	@Override
	protected void initialize() 
	{
		// TODO Auto-generated method stub

	}

	@Override
	protected void execute() 
	{
		double throttle = stick.yGet();
		double turn = Math.pow(stick.xGet(), 2) * Math.signum(stick.xGet());

		//			double throttle = stick.getDrivingY();
		//			double turn = stick.getRawAxis(0);
		throttle = Utils.deadband(throttle, 0.01);
		//			turn = Utils.deadband(turn, 0.3);

		throttle = Utils.accellimit(throttle, last, 0.2);
		leftValue = throttle + turn;
		rightValue = throttle - turn;

		driveSystem.set(rightValue, leftValue);

		last = throttle;

	}

	@Override
	protected boolean isFinished()
	{
		// TODO Auto-generated method stub
		return false;
	}


	@Override
	protected void end() 
	{
		// TODO Auto-generated method stub

	}

	@Override
	protected void interrupted() 
	{
		// TODO Auto-generated method stub

	}


}

class M_ArcadeDriveCommand extends Command
{
	DriveSystem3075 driveSystem;
	Joystick stick;

	private double leftValue;
	private double rightValue;

	public M_ArcadeDriveCommand(DriveSystem3075 driveSystem, Joystick stick) 
	{
		requires(driveSystem);
		this.driveSystem = driveSystem;
		this.stick = stick;
	}

	@Override
	protected void initialize() 
	{
		// TODO Auto-generated method stub

	}

	@Override
	protected void execute() 
	{
		double throttle = stick.getRawAxis(AxisType.kY.value);
		double turn = stick.getRawAxis(AxisType.kX.value);

		leftValue = throttle + turn;
		rightValue = throttle - turn;

		driveSystem.set(rightValue, leftValue);

	}

	@Override
	protected boolean isFinished()
	{
		// TODO Auto-generated method stub
		return false;
	}


	@Override
	protected void end() 
	{
		// TODO Auto-generated method stub

	}

	@Override
	protected void interrupted() 
	{
		// TODO Auto-generated method stub

	}

}

class M_TankDriveCommand extends Command
{
	DriveSystem3075 driveSystem;
	Joystick rightStick;
	Joystick leftStick;

	public M_TankDriveCommand(DriveSystem3075 driveSystem, Joystick rightStick, Joystick leftStick) 
	{
		requires(driveSystem);
		this.driveSystem = driveSystem;
		this.rightStick = rightStick;
		this.leftStick = leftStick;
	}

	@Override
	protected void initialize() 
	{
		// TODO Auto-generated method stub

	}

	@Override
	protected void execute() 
	{
		driveSystem.set(rightStick.getY(), leftStick.getY());
	}

	@Override
	protected boolean isFinished()
	{
		// TODO Auto-generated method stub
		return false;
	}


	@Override
	protected void end() 
	{
		// TODO Auto-generated method stub

	}

	@Override
	protected void interrupted() 
	{
		// TODO Auto-generated method stub

	}

}

//class M_TurnAngle extends Command
//{
//
//	MagicDriveSystem3075 driveSystem;
//
//	boolean endless;
//	double leftDistance;
//	double rightDistance;
//	double angle;
//	double maxA;
//
//	DrivingState prevState;
//
//	public M_TurnAngle(MagicDriveSystem3075 driveSystem, double angle, boolean endless)
//	{
//		requires(driveSystem);
//
//		this.driveSystem = driveSystem;
//		this.leftDistance = driveSystem.distancePerAngle * angle;
//		this.rightDistance = -driveSystem.distancePerAngle * angle;
//		this.angle = angle;
//		this.endless = endless;
//
//		//		this.maxA = Math.min(driveSystem.getLeftTurnMaxA(), driveSystem.getRightTurnMaxA());
//		this.maxA = driveSystem.get
//	}
//
//	public M_TurnAngle(DriveSystem3075 driveSystem, double angle, boolean endless, double maxA)
//	{		
//		requires(driveSystem);
//
//		this.driveSystem = driveSystem;
//		this.leftDistance = driveSystem.distancePerAngle * angle;
//		this.rightDistance = -driveSystem.distancePerAngle * angle;
//		this.angle = angle;
//		this.endless = endless;
//		this.maxA = maxA;
//	}
//
//	@Override
//	protected void initialize() 
//	{
//		Utils.print("started");
//
//
//		prevState = driveSystem.state;
//		driveSystem.reset();
//
//		driveSystem.setTurnMPValues(driveSystem.leftTurnMPValue, driveSystem.rightTurnMPValue);
//		driveSystem.setTolerance(driveSystem.angleTolerance * driveSystem.distancePerAngle);
//
//		driveSystem.rightMotor.set(rightDistance);
//		driveSystem.leftMotor.set(leftDistance);
//		driveSystem.enterState(DriveSystem3075.DrivingState.DistanceMotionProfiled);
//	}
//
//	@Override
//	protected void execute() 
//	{
//		// TODO Auto-generated method stub
//	}
//
//	@Override
//	public boolean isFinished() 
//	{
//		if(endless)
//		{
//			return false;
//		}
//
//		return true;
//	}
//
//	@Override
//	protected void end() 
//	{
//		driveSystem.enterState(this.prevState);
//	}
//
//	@Override
//	protected void interrupted() 
//	{
//		end();
//
//	}

class M_DriveDistance extends Command
{

	MagicDriveSystem3075 driveSystem;

	boolean endless;
	double leftDistance;
	double rightDistance;

	double leftMaxA, rightMaxA;

	LibPurple.systems.MagicDriveSystem3075.DrivingState prevState;


	public M_DriveDistance(MagicDriveSystem3075 driveSystem, double leftDistance, double rightDistance, boolean endless, double maxA)
	{
		requires(driveSystem);

		this.driveSystem = driveSystem;
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;
		this.leftMaxA = maxA;
		this.rightMaxA = maxA;

		this.endless = endless;


	}

	public M_DriveDistance(MagicDriveSystem3075 driveSystem, double leftDistance, double rightDistance, boolean endless, double leftmaxA, double rightMaxA)
	{
		requires(driveSystem);

		this.driveSystem = driveSystem;
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.leftMaxA = leftmaxA;
		this.rightMaxA = rightMaxA;
		this.endless = endless;

	}

	@Override
	protected void initialize() 
	{
		prevState = driveSystem.getState();
		driveSystem.reset();
		driveSystem.enterState(DriveSystem3075.DrivingState.DistanceMotionProfiled);
		driveSystem.rightMotor.set(rightDistance);
		driveSystem.leftMotor.set(leftDistance);
	}

	@Override
	protected void execute() 
	{
		// TODO Auto-generated method stub
	}

	@Override
	public boolean isFinished() 
	{
		if(endless)
		{
			return false;
		}

		return true;
	}

	@Override
	protected void end() 
	{
		driveSystem.enterState(this.prevState);
	}

	@Override
	protected void interrupted() 
	{
		end();
	}
}