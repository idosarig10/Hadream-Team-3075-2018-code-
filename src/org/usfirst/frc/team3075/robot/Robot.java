/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team3075.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Thread.State;

import org.usfirst.frc.team3075.robot.commands.ExampleCommand;
import org.usfirst.frc.team3075.robot.subsystems.Chassis;
import org.usfirst.frc.team3075.robot.subsystems.ExampleSubsystem;

import javax.swing.text.StyleContext.SmallAttributeSet;

/**
 * The VM is configured to automatically run this class, and to call the
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> latesttryupdate
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot 
{
	public static final ExampleSubsystem kExampleSubsystem
	= new ExampleSubsystem();
	public static OI oi;
	public static Chassis driveSystem = new Chassis();
	//	public static Elevator elevator = new Elevator();


	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

	@Override
	public void robotInit() {
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
<<<<<<< HEAD
<<<<<<< HEAD
	 * <p>You can add additional auto modes by adding additional commands to the
=======
	 * You can add additional auto modes by adding additional commands to the
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
	 * <p>You can add additional auto modes by adding additional commands to the
>>>>>>> latesttryupdate
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
<<<<<<< HEAD
=======
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
>>>>>>> latesttryupdate
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		driveSystem.reset();
	}

	double rightMaxV = 0;
	double leftMaxV = 0;
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		double rightCurrV = driveSystem.getRightEncoder().getRate();
		double leftCurrV = driveSystem.getLeftEncoder().getRate();

		if(Math.abs(rightCurrV) > Math.abs(rightMaxV))
			rightMaxV = rightCurrV;
		if(Math.abs(leftCurrV) > Math.abs(leftMaxV))
			leftMaxV = leftCurrV;
		logToDashBoard();
	}

	/**
>>>>>>> latesttryupdate
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

	public void logToDashBoard()
	{
		SmartDashboard.putNumber("left raw distance", driveSystem.getLeftEncoder().getDistance());
		SmartDashboard.putNumber("right raw distance",driveSystem.getRightEncoder().getDistance());
		SmartDashboard.putNumber("left  XD velicity", driveSystem.getLeftEncoder().getRate());
		SmartDashboard.putNumber("right XD velocity", driveSystem.getRightEncoder().getRate());
		SmartDashboard.putNumber("left XD max v", leftMaxV);
		SmartDashboard.putNumber("right XD max v", rightMaxV);
		if(Robot.driveSystem.getLeftMPController().getSetpoint() != null)
			SmartDashboard.putNumber("velocity setpoint", Robot.driveSystem.getLeftMPController().getSetpoint().velocity);
		//    	SmartDashboard.putNumber("position setpoint", setpoint.position);
	}
}
