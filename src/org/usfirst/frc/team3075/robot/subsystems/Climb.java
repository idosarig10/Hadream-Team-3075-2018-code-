//package org.usfirst.frc.team3075.robot.subsystems;
//
//import org.usfirst.frc.team3075.robot.commands.ManualClimb;
//
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//
//import edu.wpi.first.wpilibj.command.Subsystem;
//
///**
// *
// */
//public class Climb extends Subsystem 
//{
//
//	public WPI_TalonSRX climbMotor;
//	
//	public Climb()
//	{
//		climbMotor = new WPI_TalonSRX(8);
//	}
//	
//	public void setClimbSpeed(double speed){
//		climbMotor.set(speed);
//	}
//    public void initDefaultCommand() {
//        // Set the default command for a subsystem here.
//        //setDefaultCommand(new MySpecialCommand());
//    	setDefaultCommand(new ManualClimb());
//    }
//}
//
