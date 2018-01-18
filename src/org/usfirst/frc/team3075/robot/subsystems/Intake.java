package org.usfirst.frc.team3075.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {
	
//	public WPI_TalonSRX TopMotor;
//	public WPI_TalonSRX LowMotor;
//	public WPI_TalonSRX Conveyor;
	

	public Intake(){
//		TopMotor = new WPI_TalonSRX(RobotMap.TopMotor);
//		LowMotor = new WPI_TalonSRX(RobotMap.LowMotor);
	}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
//    	public void setspeed(double speed)
//    	{
//    		TopMotor.set(speed);
//    		LowMotor.set(speed);
//    	}
    }
}