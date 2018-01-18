package LibPurple.systems;


import org.usfirst.frc.team3075.robot.Robot;

import LibPurple.control.MPController;
import LibPurple.control.MPController.MPValue;
import LibPurple.control.PIDvalue;
<<<<<<< HEAD
import LibPurple.control.TrajectoryFile;
import LibPurple.control.TrajectorySMP;
<<<<<<< HEAD
=======
import LibPurple.sensors.Camera3075;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
import LibPurple.control.Trajectory3075;
import LibPurple.control.Trajectory3075.Type;
import LibPurple.control.TrajectoryFile;
import LibPurple.control.TrajectorySMP;
import LibPurple.control.TrajectoryTMP;
>>>>>>> latesttryupdate
import LibPurple.sensors.ConsoleJoystick;
import LibPurple.sensors.Encoder3075;
import LibPurple.systems.DriveSystem3075.DrivingState;
import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.tables.ITable;
/***
 * Initialization example:
 * super.setPIDValues(...);
 * super.setMPValues(...);
 * super.setTurnMPValues(...);
 * super.setVelocityTolerance(...);
 * super.setDistanceTolerance(...);
 * super.distancePerAngle = ...
 * super.rightMaxV = ...
 * super.leftMaxV = ...
 * super.leftMaxA = ...
 * super.rightMaxA = ...
 * 
 * @author 3075
 *
 */
public abstract class DriveSystem3075 extends Subsystem implements Sendable
{
	public static enum DrivingState
	{
		Voltage, VelocityClosedLoop, DistanceMotionProfiled;
	}
<<<<<<< HEAD
	
	DrivingState state = DrivingState.Voltage;

//	protected Camera3075 camera;
	
	protected SpeedController rightMotor;
	protected SpeedController leftMotor;
	
	private Encoder3075 rightEncoder;
	private Encoder3075 leftEncoder;
	
	private PIDController rightPID;
	protected PIDvalue rightPIDValue;
	
	private PIDController leftPID;
	private PIDvalue leftPIDValue;
	
=======

	DrivingState state = DrivingState.Voltage;

	//	protected Camera3075 camera;

	protected SpeedController rightMotor;
	protected SpeedController leftMotor;

	private Encoder3075 rightEncoder;
	private Encoder3075 leftEncoder;

	private PIDController rightPID;
	protected PIDvalue rightPIDValue;

	private PIDController leftPID;
	private PIDvalue leftPIDValue;

>>>>>>> latesttryupdate
	protected MPController rightMP;
	protected MPValue rightMPValue;
	protected MPValue rightTurnMPValue;
	protected MPController leftMP;
	protected MPValue leftMPValue;
	protected MPValue leftTurnMPValue;
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
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
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate

	protected void initialize(SpeedController rightMotor, SpeedController leftMotor,
			Encoder3075 rightEncoder, Encoder3075 leftEncoder)
	{
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.rightEncoder = rightEncoder;
		this.leftEncoder = leftEncoder;
<<<<<<< HEAD
		
		this.rightEncoder.setPIDSourceType(PIDSourceType.kRate);
		this.leftEncoder.setPIDSourceType(PIDSourceType.kRate);
		
		leftMP = new MPController(leftMPValue, leftMotor, leftEncoder);
		rightMP = new MPController(rightMPValue, rightMotor, rightEncoder);
		
	}
	
=======

		this.rightEncoder.setPIDSourceType(PIDSourceType.kRate);
		this.leftEncoder.setPIDSourceType(PIDSourceType.kRate);

		leftMP = new MPController(leftMPValue, leftMotor, leftEncoder);
		rightMP = new MPController(rightMPValue, rightMotor, rightEncoder);
	}

>>>>>>> latesttryupdate

	@Override
	protected void initDefaultCommand()
	{
		// TODO Auto-generated method stub
	}
	/**
	 * if the drive system is in velocity state then sets the system's speed
	 * if the drive system is in voltage state then gives the values straight to the engines
	 * @param rightValue - the right engine speed/value
	 * @param leftValue - the left engine speed/value
	 */
	public void set(double rightValue, double leftValue)
	{
		if(state == DrivingState.Voltage)
		{
			rightMotor.set(rightValue);
			leftMotor.set(leftValue);
		}
		else if(state == DrivingState.VelocityClosedLoop)
		{
			double maxV = Math.min(rightMaxV, leftMaxV);
<<<<<<< HEAD
			
=======

>>>>>>> latesttryupdate
			if(rightValue == 0)
			{
				rightPID.disable();
				rightPID.reset();
			}
			else
				rightPID.enable();	
<<<<<<< HEAD
			
=======

>>>>>>> latesttryupdate
			if(leftValue == 0)
			{
				leftPID.disable();
				leftPID.reset();
			}
			else
				leftPID.enable();
<<<<<<< HEAD
			
=======

>>>>>>> latesttryupdate
			rightPID.setSetpoint(rightValue * maxV);
			leftPID.setSetpoint(leftValue * maxV);
		}
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	/**
	 * switched the drive system state from its current state to a new state 
	 * @param newState - the new state, can get "Voltage", "VelocityClosedLoop","DistanceMotionProfiled" 
	 */
	public void enterState(DrivingState newState)
	{
		this.state = newState;
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
		switch(state)
		{
		case Voltage:
			setPIDEnabled(false);
			setMPEnabled(false);
			break;
		case VelocityClosedLoop:
			rightEncoder.setPIDSourceType(PIDSourceType.kRate);
			leftEncoder.setPIDSourceType(PIDSourceType.kRate);
			setPIDEnabled(true);
			setMPEnabled(false);
			break;
		case DistanceMotionProfiled:
			rightEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
			leftEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
			setPIDEnabled(false);
			setMPEnabled(true);
<<<<<<< HEAD
=======
			Utils.print("mp enabled");
>>>>>>> latesttryupdate
			break;
		default:
			setPIDEnabled(false);
			setMPEnabled(false);
			break;
<<<<<<< HEAD
			
		}
	}
	
=======

		}
	}

>>>>>>> latesttryupdate
	/**
	 * enable or disables the PID
	 * @param enabled - boolean that sets the PID
	 */
	private void setPIDEnabled(boolean enabled)
	{
		if(enabled)
		{
			rightPID.enable();
			leftPID.enable();
		}
		else
		{
			rightPID.disable();
			leftPID.disable();
		}
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	/**
	 * enable or disable the MP
	 * @param enabled - boolean that sets the MP
	 */
	private void setMPEnabled(boolean enabled)
	{
		if(enabled)
		{
			leftMP.enable();
			rightMP.enable();
		}
		else
		{
			rightMP.disable();
			leftMP.disable();
		}
<<<<<<< HEAD
		
	}
	
=======

	}

>>>>>>> latesttryupdate
	/**
	 * resets the encoders
	 */
	public void reset()
	{
		leftEncoder.reset();
		rightEncoder.reset();
		leftPID.reset();
		rightPID.reset();
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public Command arcadeDrive(Joystick joystick)
	{
		return new ArcadeDriveCommand(this, joystick);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public Command xboxArcadeDrive(ConsoleJoystick stick)
	{
		return new XboxArcade(this, stick);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public Command tankDrive(Joystick rightJoystick, Joystick leftJoystick)
	{
		return new TankDriveCommand(this, rightJoystick, leftJoystick);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	/**
	 * Toggles the drive system state between the two states given.
	 * 
	 * @param state1 - the first state
	 * @param state2 - the second state
	 */
	public Command stateToggle(DrivingState state1, DrivingState state2)
	{
		return new StateToggle(this, state1, state2);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	/**
	 * drives straight for a given distance
	 * @param distance - the distance to drive straight
	 */
	public Command driveStraight(double distance)
	{
		double maxA = Math.min(rightMaxA, leftMaxA);
		return new DriveDistance(this, distance, distance, false, maxA);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	@Deprecated
	/**
	 * drives straight for a given distance but with a limit on the acceleration
	 * @param distance - the distance to drive straight
	 * @param maxA - the maximum acceleration that the robot will reach during the trajectory
	 */
	public Command driveStraight(double distance, double maxA)
	{
		return new DriveDistance(this, distance, distance, false, maxA);
	}
<<<<<<< HEAD
	
	
=======


>>>>>>> latesttryupdate
	/**
	 * drives straight for a given distance, does not stop the command when the distance is reached
	 * @param distance - the distance to drive
	 */
	public Command driveStraightEndless(double distance)
	{
		double maxA = Math.min(rightMaxA, leftMaxA);
		return new DriveDistance(this, distance, distance, true, maxA);
	}
	
<<<<<<< HEAD
=======
	public Command driveStraightTrapizodial(double distance, boolean endless)
	{
		return new DriveDistance(this, distance, distance, endless, getMaxA(), getMaxA(), Type.TrapizoidalMotionProfile, getMaxV(), getMaxV());
	}
	

>>>>>>> latesttryupdate
	/**
	 * drives in a circular motion
	 * @param radius - the radius of the circle 
	 * @param angle - the angle of the motion, positive angle is left, negative is right
<<<<<<< HEAD
	 */
	public Command driveCurve(double radius, double angle, boolean clockwise)
	{
		double leftRadius = clockwise ? radius +  (robotWidth / 2) : radius - (robotWidth / 2);
		double rightRadius = clockwise ? radius -  (robotWidth / 2) : radius + (robotWidth / 2);
		
		double leftDistance = Math.toRadians(angle) * leftRadius; 
		double rightDistance = Math.toRadians(angle) * rightRadius;
				
		double leftMaxA = !clockwise ? 
				getMaxA() * (Math.min(leftRadius, rightRadius) / Math.max(leftRadius, rightRadius)) 
				: getMaxA(); 
		double rightMaxA = clockwise ? 
				getMaxA() * (Math.min(leftRadius, rightRadius) / Math.max(leftRadius, rightRadius)) 
				: getMaxA(); 
		
		
		return new DriveDistance(this, leftDistance, rightDistance, false, leftMaxA, rightMaxA);
	}
	
=======
	 * @param clockwise - true for driving on clockwise direction on the circle.
	 */
	public Command driveArc(double radius, double angle, boolean clockwise)
	{
		double leftRadius = clockwise ? radius +  (robotWidth / 2) : radius - (robotWidth / 2); // robot's left side circle radius
		double rightRadius = clockwise ? radius -  (robotWidth / 2) : radius + (robotWidth / 2); //robot's right side circle radius

		double leftDistance = Math.toRadians(angle) * leftRadius; //robot's left side distance 
		double rightDistance = Math.toRadians(angle) * rightRadius;//robot's right side distance

		//robot's left side max acceleration
		double leftMaxA = !clockwise ? getMaxA() * (Math.min(leftRadius, rightRadius) / Math.max(leftRadius, rightRadius)) : getMaxA();
		//robot's right side max acceleration
		double rightMaxA = clockwise ? getMaxA() * (Math.min(leftRadius, rightRadius) / Math.max(leftRadius, rightRadius)) : getMaxA();

		//robot's left side max velocity
		double leftMaxV = !clockwise ? getMaxV() * (Math.min(leftRadius, rightRadius) / Math.max(leftRadius, rightRadius)) : getMaxV();
		//robot's right side max velocity
		double rightMaxV = clockwise ? getMaxV() * (Math.min(leftRadius, rightRadius) / Math.max(leftRadius, rightRadius)) : getMaxV();
		
		Utils.print("left max v:" + (leftMaxV/2));
		Utils.print("right max v: " + (rightMaxV/2));
		
		return new DriveDistance(this, leftDistance, rightDistance, false, leftMaxA/2, rightMaxA/2, Type.TrapizoidalMotionProfile, leftMaxV/2, rightMaxV/2);
	}

>>>>>>> latesttryupdate
	/**
	 * drives a specific path, using given files
	 * @param rightFile - the file to the right engine
	 * @param leftFile - the file to the left engine
	 * @return
	 */
	public Command drive2D(String rightFile, String leftFile)
	{
		return new Drive2D(this, rightFile, leftFile, false, false);
	}
<<<<<<< HEAD
	
	public Command drive2D(String rightFile, String leftFile, boolean reversed)
	{
			if(reversed)
				return new Drive2D(this, leftFile, rightFile, false, reversed);
			else
				return new Drive2D(this, rightFile, leftFile, false, reversed);
	}
	
=======

	public Command drive2D(String rightFile, String leftFile, boolean reversed)
	{
		if(reversed)
			return new Drive2D(this, leftFile, rightFile, false, reversed);
		else
			return new Drive2D(this, rightFile, leftFile, false, reversed);
	}

>>>>>>> latesttryupdate
	/**
	 * turns a given angle, stays in the same position
	 * @param angle - the angle to turn
	 * @return
	 */
	public Command turnAngle(double angle)
	{			
		return new TurnAngle(this, angle,  false);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public Command turnAngle(double angle, double maxA)
	{			
		return new TurnAngle(this, angle,  false, maxA);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	/**
	 * turns a given angle, stays in the same position, does not stop the command when the angle is reached
	 * @param angle
	 * @return
	 */
	public Command turnAngleEndless(double angle)
	{
		return new TurnAngle(this, angle,  true);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	/**
	 * sets the PID values
	 * @param leftPIDValue - the values for the left side
	 * @param rightPIDValue - the values for the right side
	 */
	public void setPIDValues(PIDvalue leftPIDValue, PIDvalue rightPIDValue) 
	{
		this.leftPIDValue = leftPIDValue;
		this.rightPIDValue = rightPIDValue;
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
		if(this.leftPID != null)
			leftPID.setPID(leftPIDValue.kP, leftPIDValue.kI, leftPIDValue.kD, leftPIDValue.kF);
		else
			leftPID = new PIDController(leftPIDValue.kP, leftPIDValue.kI, leftPIDValue.kD, leftPIDValue.kF, leftEncoder, leftMotor);

		if(this.rightPID != null)
			rightPID.setPID(rightPIDValue.kP, rightPIDValue.kI, rightPIDValue.kD, rightPIDValue.kF);
		else
			rightPID = new PIDController(rightPIDValue.kP, rightPIDValue.kI, rightPIDValue.kD, rightPIDValue.kF, rightEncoder, rightMotor);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	/**
	 * sets the MP values
	 * @param leftMPValue - the values for the left side
	 * @param rightMPValue - the values for the right side
	 */
	public void setMPValues(MPValue leftMPValue, MPValue rightMPValue)
	{
		this.leftMPValue = leftMPValue;
		this.rightMPValue = rightMPValue;
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
		leftMP.setValues(leftMPValue);
		rightMP.setValues(rightMPValue);
	}

	/**
	 * sets the tolerance for the velocity
	 * @param velocityTolerance - the tolerance to set
	 */
	public void setVelocityTolerance(double velocityTolerance) 
	{
		leftPID.setAbsoluteTolerance(velocityTolerance);
		rightPID.setAbsoluteTolerance(velocityTolerance);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	/**
	 * sets the tolerance for the distance
	 * @param tolerance - the tolerance to set
	 */
	public void setTolerance(double tolerance) 
	{
		rightMP.setTolerance(tolerance);
		leftMP.setTolerance(tolerance);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public double getAngle()
	{
		return (leftEncoder.getDistance() - rightEncoder.getDistance()) / (2 * distancePerAngle);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public PIDvalue getLeftPIDValue() 
	{
		return leftPIDValue;
	}

	public PIDvalue getRightPIDValue()
	{
		return rightPIDValue;
	}

	public MPValue getRightMPValue()
	{
		return rightMPValue;
	}

	public MPValue getRightTurnMPValue()
	{
		return rightTurnMPValue;
	}

	public MPValue getLeftMPValue() 
	{
		return leftMPValue;
	}

	public MPValue getLeftTurnMPValue()
	{
		return leftTurnMPValue;
	}

	public void setTurnMPValues(MPValue leftTurnMPValue, MPValue rightTurnMPValue) 
	{
		this.leftTurnMPValue = leftTurnMPValue;
		this.rightTurnMPValue = rightTurnMPValue;
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
		leftMP.setValues(leftTurnMPValue);
		rightMP.setValues(rightTurnMPValue);
	}

	public double getRightMaxV()
	{
		return rightMaxV;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public double getLeftTurnMaxA()
	{
		return leftTurnMaxA;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public double getRightTurnMaxA()
	{
		return rightTurnMaxA;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public double getLeftTurnMaxV()
	{
		return leftTurnMaxV;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public double getRightTurnMaxV()
	{
		return rightTurnMaxV;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public void setMaxV(double rightMaxV, double leftMaxV)
	{
		this.rightMaxV = rightMaxV;
		this.leftMaxV = leftMaxV;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public void setTurnMaxV(double rightTurnMaxV, double leftTurnMaxV)
	{
		this.rightTurnMaxV = rightTurnMaxV;
		this.leftTurnMaxV = leftTurnMaxV;
	}

	public double getLeftMaxV()
	{
		return leftMaxV;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public void setMaxA(double rightMaxA, double leftMaxA)
	{
		this.rightMaxA = rightMaxA;
		this.leftMaxA = leftMaxA;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public void setTurnMaxA(double rightTurnMaxA, double leftTurnMaxA)
	{
		this.rightTurnMaxA = rightTurnMaxA;
		this.leftTurnMaxA = leftTurnMaxA;
	}
<<<<<<< HEAD
	
	
=======


>>>>>>> latesttryupdate
	public double getRightMaxA() {
		return rightMaxA;
	}

	public double getLeftMaxA() {
		return leftMaxA;
	}

	public MPController getRightMPController()
	{
		return rightMP;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public MPController getLeftMPController()
	{
		return leftMP;
	}

	public double getDistanceTolerance() 
	{
		return rightMP.getTolerance();
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public double getMaxA(){
		return Math.min(this.leftMaxA, this.rightMaxA);
	}
	
<<<<<<< HEAD
=======
	public double getMaxV()
	{
		return Math.min(leftMaxV, rightMaxV);
	}

>>>>>>> latesttryupdate
	public DrivingState getState(){
		return state;
	}

	public double getDistancePerAngle() 
	{
		return distancePerAngle;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public double getRobotWidth()
	{
		return robotWidth;
	}

	public void setDistancePerAngle(double distancePerAngle) 
	{
		this.distancePerAngle = distancePerAngle;
	}

	public double getAverageError() 
	{
		return (rightPID.getError() + leftPID.getError()) / 2;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	public double getAngleTolerance()
	{
		return angleTolerance;
	}
<<<<<<< HEAD
	
	private ITable m_table;
	
	public void initTable(ITable subtable) {
		m_table = subtable;
		updateTable();
		
=======

	private ITable m_table;

	public void initTable(ITable subtable) {
		m_table = subtable;
		updateTable();

>>>>>>> latesttryupdate
	}

	public ITable getTable() {
		// TODO Auto-generated method stub
		return m_table;
	}

	public String getSmartDashboardType() {
		// TODO Auto-generated method stub
		return "Drive System";
	}
<<<<<<< HEAD
	
	public void updateTable() 
	{
	    if (m_table != null) 
	    {
	      m_table.putNumber("Right Rate", rightEncoder.getRate());
	      m_table.putNumber("Left Rate", leftEncoder.getRate());
	      m_table.putNumber("Right Distance", rightEncoder.getDistance());
	      m_table.putNumber("Left Distance",  leftEncoder.getDistance());
	      m_table.putNumber("Right Rate Setpoint", rightPID.getSetpoint());
	      m_table.putNumber("Left Rate Setpoint", leftPID.getSetpoint());
	      m_table.putNumber("Angle", getAngle());
	      m_table.putString("Driving State", state + "");
	      
	    }
=======

	public void updateTable() 
	{
		if (m_table != null) 
		{
			m_table.putNumber("Right Rate", rightEncoder.getRate());
			m_table.putNumber("Left Rate", leftEncoder.getRate());
			m_table.putNumber("Right Distance", rightEncoder.getDistance());
			m_table.putNumber("Left Distance",  leftEncoder.getDistance());
			m_table.putNumber("Right Rate Setpoint", rightPID.getSetpoint());
			m_table.putNumber("Left Rate Setpoint", leftPID.getSetpoint());
			m_table.putNumber("Angle", getAngle());
			m_table.putString("Driving State", state + "");

		}
>>>>>>> latesttryupdate
	}
}

class XboxArcade extends Command
{
<<<<<<< HEAD
	
	DriveSystem3075 driveSystem;
	ConsoleJoystick stick;
	
	private double leftValue;
    private double rightValue;
    private double last = 0;
	
=======

	DriveSystem3075 driveSystem;
	ConsoleJoystick stick;

	private double leftValue;
	private double rightValue;
	private double last = 0;

>>>>>>> latesttryupdate
	public XboxArcade(DriveSystem3075 driveSystem, ConsoleJoystick stick) {
		requires(driveSystem);
		this.driveSystem = driveSystem;
		this.stick = stick;
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	@Override
	protected void initialize() 
	{
		// TODO Auto-generated method stub
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
	}

	@Override
	protected void execute() 
	{
		double throttle = stick.yGet();
		double turn = Math.pow(stick.xGet(), 2) * Math.signum(stick.xGet());
<<<<<<< HEAD
		
//		double throttle = stick.getDrivingY();
//		double turn = stick.getRawAxis(0);
		throttle = Utils.deadband(throttle, 0.01);
//		turn = Utils.deadband(turn, 0.3);
		
		throttle = Utils.accellimit(throttle, last, 0.2);
        leftValue = (throttle + turn);
        rightValue = (throttle - turn);

        driveSystem.set(rightValue, leftValue);
        
        last = throttle;
        
	}
	
=======

		//		double throttle = stick.getDrivingY();
		//		double turn = stick.getRawAxis(0);
		throttle = Utils.deadband(throttle, 0.01);
		//		turn = Utils.deadband(turn, 0.3);

		throttle = Utils.accellimit(throttle, last, 0.2);
		leftValue = (throttle + turn);
		rightValue = (throttle - turn);

		driveSystem.set(rightValue, leftValue);

		last = throttle;

	}

>>>>>>> latesttryupdate
	@Override
	protected boolean isFinished()
	{
		// TODO Auto-generated method stub
		return false;
	}
<<<<<<< HEAD
	
	
=======


>>>>>>> latesttryupdate
	@Override
	protected void end() 
	{
		// TODO Auto-generated method stub
<<<<<<< HEAD
		
	}
	
=======

	}

>>>>>>> latesttryupdate
	@Override
	protected void interrupted() 
	{
		// TODO Auto-generated method stub
<<<<<<< HEAD
		
	}

	
=======

	}


>>>>>>> latesttryupdate
}

class ArcadeDriveCommand extends Command
{
	DriveSystem3075 driveSystem;
	Joystick stick;
<<<<<<< HEAD
	
	private double leftValue;
    private double rightValue;
    
=======

	private double leftValue;
	private double rightValue;

>>>>>>> latesttryupdate
	public ArcadeDriveCommand(DriveSystem3075 driveSystem, Joystick stick) 
	{
		requires(driveSystem);
		this.driveSystem = driveSystem;
		this.stick = stick;
	}

	@Override
	protected void initialize() 
	{
		// TODO Auto-generated method stub
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
	}

	@Override
	protected void execute() 
	{
		double throttle = stick.getRawAxis(AxisType.kY.value);
		double turn = stick.getRawAxis(AxisType.kX.value);
<<<<<<< HEAD
		
        leftValue = throttle + turn;
        rightValue = throttle - turn;

        driveSystem.set(rightValue, leftValue);
        
	}
	
=======

		leftValue = throttle + turn;
		rightValue = throttle - turn;

		driveSystem.set(rightValue, leftValue);

	}

>>>>>>> latesttryupdate
	@Override
	protected boolean isFinished()
	{
		// TODO Auto-generated method stub
		return false;
	}
<<<<<<< HEAD
	
	
=======


>>>>>>> latesttryupdate
	@Override
	protected void end() 
	{
		// TODO Auto-generated method stub
<<<<<<< HEAD
		
	}
	
=======

	}

>>>>>>> latesttryupdate
	@Override
	protected void interrupted() 
	{
		// TODO Auto-generated method stub
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
	}

}

class TankDriveCommand extends Command
{
	DriveSystem3075 driveSystem;
	Joystick rightStick;
	Joystick leftStick;
<<<<<<< HEAD
    
=======

>>>>>>> latesttryupdate
	public TankDriveCommand(DriveSystem3075 driveSystem, Joystick rightStick, Joystick leftStick) 
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
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
	}

	@Override
	protected void execute() 
	{
<<<<<<< HEAD
        driveSystem.set(rightStick.getY(), leftStick.getY());
	}
	
=======
		driveSystem.set(rightStick.getY(), leftStick.getY());
	}

>>>>>>> latesttryupdate
	@Override
	protected boolean isFinished()
	{
		// TODO Auto-generated method stub
		return false;
	}
<<<<<<< HEAD
	
	
=======


>>>>>>> latesttryupdate
	@Override
	protected void end() 
	{
		// TODO Auto-generated method stub
<<<<<<< HEAD
		
	}
	
=======

	}

>>>>>>> latesttryupdate
	@Override
	protected void interrupted() 
	{
		// TODO Auto-generated method stub
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
	}

}

class TurnAngle extends Command
{
<<<<<<< HEAD
	
	DriveSystem3075 driveSystem;
	MPController rightMP;
	MPController leftMP;
	
=======

	DriveSystem3075 driveSystem;
	MPController rightMP;
	MPController leftMP;

>>>>>>> latesttryupdate
	boolean endless;
	double leftDistance;
	double rightDistance;
	double angle;
	double maxA;
<<<<<<< HEAD
	
	DrivingState prevState;
	
	public TurnAngle(DriveSystem3075 driveSystem, double angle, boolean endless)
	{
		requires(driveSystem);
		
=======

	DrivingState prevState;

	public TurnAngle(DriveSystem3075 driveSystem, double angle, boolean endless)
	{
		requires(driveSystem);

>>>>>>> latesttryupdate
		this.driveSystem = driveSystem;
		this.leftDistance = driveSystem.distancePerAngle * angle;
		this.rightDistance = -driveSystem.distancePerAngle * angle;
		this.angle = angle;
		this.endless = endless;
<<<<<<< HEAD
		
		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();
		
		this.maxA = Math.min(driveSystem.getLeftTurnMaxA(), driveSystem.getRightTurnMaxA());		
	}
	
	public TurnAngle(DriveSystem3075 driveSystem, double angle, boolean endless, double maxA)
	{		
		requires(driveSystem);
		
=======

		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();

		this.maxA = Math.min(driveSystem.getLeftTurnMaxA(), driveSystem.getRightTurnMaxA());		
	}

	public TurnAngle(DriveSystem3075 driveSystem, double angle, boolean endless, double maxA)
	{		
		requires(driveSystem);

>>>>>>> latesttryupdate
		this.driveSystem = driveSystem;
		this.leftDistance = driveSystem.distancePerAngle * angle;
		this.rightDistance = -driveSystem.distancePerAngle * angle;
		this.angle = angle;
		this.endless = endless;
		this.maxA = maxA;
<<<<<<< HEAD
		
		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();
	}
	
=======

		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();
	}

>>>>>>> latesttryupdate
	@Override
	protected void initialize() 
	{
		Utils.print("started");

<<<<<<< HEAD
		
		prevState = driveSystem.state;
		driveSystem.reset();
		
		driveSystem.setTurnMPValues(driveSystem.leftTurnMPValue, driveSystem.rightTurnMPValue);
		driveSystem.setTolerance(driveSystem.angleTolerance * driveSystem.distancePerAngle);
		
=======

		prevState = driveSystem.state;
		driveSystem.reset();

		driveSystem.setTurnMPValues(driveSystem.leftTurnMPValue, driveSystem.rightTurnMPValue);
		driveSystem.setTolerance(driveSystem.angleTolerance * driveSystem.distancePerAngle);

>>>>>>> latesttryupdate
		rightMP.setTrajectory(new TrajectorySMP(rightDistance, maxA));
		leftMP.setTrajectory(new TrajectorySMP(leftDistance, maxA));
		driveSystem.enterState(DriveSystem3075.DrivingState.DistanceMotionProfiled);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
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
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
		return Utils.inRange(driveSystem.getAngle(), this.angle, driveSystem.angleTolerance);
	}

	@Override
	protected void end() 
	{
		leftMP.disable();
		rightMP.disable();
		driveSystem.enterState(this.prevState);
	}

	@Override
	protected void interrupted() 
	{
		end();
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
}

class DriveDistance extends Command
{
<<<<<<< HEAD
	
	DriveSystem3075 driveSystem;
	MPController rightMP;
	MPController leftMP;
	
	boolean endless;
	double leftDistance;
	double rightDistance;
	
	double leftMaxA, rightMaxA;
	
	DrivingState prevState;

	
	public DriveDistance(DriveSystem3075 driveSystem, double leftDistance, double rightDistance, boolean endless, double maxA)
	{
		requires(driveSystem);
		
=======

	DriveSystem3075 driveSystem;
	MPController rightMP;
	MPController leftMP;

	boolean endless;
	double leftDistance;
	double rightDistance;

	double leftMaxA, rightMaxA;
	
	double leftMaxV, rightMaxV;

	DrivingState prevState;

	Trajectory3075.Type MPType;


	public DriveDistance(DriveSystem3075 driveSystem, double leftDistance, double rightDistance, boolean endless, double maxA)
	{
		requires(driveSystem);

		this.MPType = Type.SinosoidalMotionProfile;
>>>>>>> latesttryupdate
		this.driveSystem = driveSystem;
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;
		this.leftMaxA = maxA;
		this.rightMaxA = maxA;
<<<<<<< HEAD
		
		this.endless = endless;
		
		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();
		
	}
	
	public DriveDistance(DriveSystem3075 driveSystem, double leftDistance, double rightDistance, boolean endless, double leftmaxA, double rightMaxA)
	{
		requires(driveSystem);
		
		this.driveSystem = driveSystem;
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;
		
		this.leftMaxA = leftmaxA;
		this.rightMaxA = rightMaxA;
		this.endless = endless;
		
		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();
	}
	
=======

		this.endless = endless;

		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();

	}

	public DriveDistance(DriveSystem3075 driveSystem, double leftDistance, double rightDistance, boolean endless, double leftmaxA, double rightMaxA)
	{
		requires(driveSystem);

		this.MPType = Type.SinosoidalMotionProfile;
		this.driveSystem = driveSystem;
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.leftMaxA = leftmaxA;
		this.rightMaxA = rightMaxA;
		this.endless = endless;

		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();
	}

	public DriveDistance(DriveSystem3075 driveSystem, double leftDistance, double rightDistance, boolean endless, double leftmaxA, double rightMaxA, Type MPType, double leftMaxV, double rightMaxV)
	{
		requires(driveSystem);

		this.MPType = MPType;
		this.driveSystem = driveSystem;
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.leftMaxA = leftmaxA;
		this.rightMaxA = rightMaxA;
		
		this.leftMaxV = leftMaxV;
		this.rightMaxV = rightMaxV;
		
		this.endless = endless;

		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();
	}


>>>>>>> latesttryupdate
	@Override
	protected void initialize() 
	{
		prevState = driveSystem.state;
		driveSystem.reset();
<<<<<<< HEAD
		
		driveSystem.setMPValues(driveSystem.leftMPValue, driveSystem.rightMPValue);
		driveSystem.setTolerance(driveSystem.positionTolerance);
		
		rightMP.setTrajectory(new TrajectorySMP(rightDistance, rightMaxA));
		leftMP.setTrajectory(new TrajectorySMP(leftDistance, leftMaxA));
		driveSystem.enterState(DriveSystem3075.DrivingState.DistanceMotionProfiled);
	}
	
=======

		driveSystem.setMPValues(driveSystem.leftMPValue, driveSystem.rightMPValue);
		driveSystem.setTolerance(driveSystem.positionTolerance);

		if(this.MPType == Type.SinosoidalMotionProfile)
		{
			rightMP.setTrajectory(new TrajectorySMP(rightDistance, rightMaxA));
			leftMP.setTrajectory(new TrajectorySMP(leftDistance, leftMaxA));
		}
		else if(this.MPType == Type.TrapizoidalMotionProfile)
		{
			rightMP.setTrajectory(new TrajectoryTMP(rightDistance, rightMaxA, rightMaxV));
			leftMP.setTrajectory(new TrajectoryTMP(leftDistance, leftMaxA, leftMaxV));
		}
		driveSystem.enterState(DriveSystem3075.DrivingState.DistanceMotionProfiled);
	}

>>>>>>> latesttryupdate
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
<<<<<<< HEAD
		
=======

>>>>>>> latesttryupdate
		return leftMP.isTimeUp() && rightMP.isTimeUp() && rightMP.onTarget() && leftMP.onTarget();
	}

	@Override
	protected void end() 
	{
		leftMP.disable();
		rightMP.disable();
		driveSystem.enterState(this.prevState);
	}

	@Override
	protected void interrupted() 
	{
		end();
	}
<<<<<<< HEAD

	
=======
>>>>>>> latesttryupdate
}


class Drive2D extends Command
{
<<<<<<< HEAD
	
	DriveSystem3075 driveSystem;
	MPController rightMP;
	MPController leftMP;
	
=======

	DriveSystem3075 driveSystem;
	MPController rightMP;
	MPController leftMP;

>>>>>>> latesttryupdate
	boolean endless;
	boolean reversed;
	String rightMotorFile;
	String leftMotorFile;
<<<<<<< HEAD
	
	DrivingState prevState;

	
	public Drive2D(DriveSystem3075 driveSystem, String rightMotorFile, String leftMotorFile, boolean endless, boolean reversed)
	{
		requires(driveSystem);
		
=======

	DrivingState prevState;


	public Drive2D(DriveSystem3075 driveSystem, String rightMotorFile, String leftMotorFile, boolean endless, boolean reversed)
	{
		requires(driveSystem);

>>>>>>> latesttryupdate
		this.driveSystem = driveSystem;
		this.rightMotorFile = rightMotorFile;
		this.leftMotorFile = leftMotorFile;
		this.endless = endless;
		this.reversed = reversed;
<<<<<<< HEAD
		
		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();
		
	}
	
=======

		rightMP = driveSystem.getRightMPController();
		leftMP = driveSystem.getLeftMPController();

	}

>>>>>>> latesttryupdate
	@Override
	protected void initialize() 
	{

		prevState = driveSystem.state;
		driveSystem.reset();
<<<<<<< HEAD
		
		driveSystem.setMPValues(driveSystem.leftMPValue, driveSystem.rightMPValue);
		driveSystem.setTolerance(driveSystem.positionTolerance);
		
=======

		driveSystem.setMPValues(driveSystem.leftMPValue, driveSystem.rightMPValue);
		driveSystem.setTolerance(driveSystem.positionTolerance);

>>>>>>> latesttryupdate
		rightMP.setTrajectory(new TrajectoryFile(rightMotorFile, reversed));
		leftMP.setTrajectory(new TrajectoryFile(leftMotorFile, reversed));
		driveSystem.enterState(DriveSystem3075.DrivingState.DistanceMotionProfiled);
	}
<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
	@Override
	protected void execute() 
	{
		// TODO Auto-generated method stub
	}

	@Override
	protected boolean isFinished() 
	{
		if(endless)
		{
			return false;
		}
<<<<<<< HEAD
		
		return rightMP.isTimeUp();
	}
	
=======

		return rightMP.isTimeUp();
	}

>>>>>>> latesttryupdate
	@Override
	protected void end() 
	{
		leftMP.disable();
		rightMP.disable();
		driveSystem.enterState(this.prevState);
		Utils.print("2D Done!");
	}

	@Override
	protected void interrupted() 
	{
		end();
	}

<<<<<<< HEAD
	
=======

>>>>>>> latesttryupdate
}

class StateToggle extends InstantCommand
{
	DriveSystem3075.DrivingState state1, state2;
	DriveSystem3075 driveSystem;
<<<<<<< HEAD
	

	public StateToggle(DriveSystem3075 driveSystem, DrivingState state1, DrivingState state2) 
	{
//		requires(driveSystem);
=======


	public StateToggle(DriveSystem3075 driveSystem, DrivingState state1, DrivingState state2) 
	{
		//		requires(driveSystem);
>>>>>>> latesttryupdate
		this.driveSystem = driveSystem;
		this.state1 = state1;
		this.state2 = state2;
	}

	@Override
	protected void initialize() 
	{	
		if(driveSystem.getState().equals(state1))
		{
			driveSystem.enterState(state2);
		}
		else
		{
			driveSystem.enterState(state1);
		}
	}
}

//class CameraTurn extends Command
//{
//	private double setPoint;
////	private PIDController pid;
//	
//	public CameraTurn(double setPoint)
//	{
//		this.setPoint = setPoint;
//	}
//	
//	@Override
//	protected void initialize()
//	{
//	}
//	
//	@Override
//	protected void execute() {
//		if (Double.parseDouble(Robot.server.getString()) > setPoint)
//		{
//			
//		}
//	}
//		
//	@Override
//	protected boolean isFinished() {
//		// TODO Auto-generated method stub
//		return false;
//	}
//	
//}
