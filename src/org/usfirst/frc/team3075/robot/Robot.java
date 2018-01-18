<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> latesttryupdate
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
<<<<<<< HEAD
=======
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
>>>>>>> latesttryupdate

package org.usfirst.frc.team3075.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> latesttryupdate
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Thread.State;

<<<<<<< HEAD
import org.usfirst.frc.team3075.robot.commands.ExampleCommand;
import org.usfirst.frc.team3075.robot.subsystems.Chassis;
=======
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3075.robot.commands.ExampleCommand;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
import javax.swing.text.StyleContext.SmallAttributeSet;

import org.usfirst.frc.team3075.robot.commands.ExampleCommand;
import org.usfirst.frc.team3075.robot.subsystems.Chassis;
<<<<<<< HEAD
>>>>>>> latesttryupdate
=======
//import org.usfirst.frc.team3075.robot.subsystems.Elevator;
>>>>>>> latesttryupdate
import org.usfirst.frc.team3075.robot.subsystems.ExampleSubsystem;

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
<<<<<<< HEAD
	public static OI m_oi;
=======
	public static OI oi;
>>>>>>> latesttryupdate
	public static Chassis driveSystem = new Chassis();
//	public static Elevator elevator = new Elevator();
	
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
<<<<<<< HEAD
=======
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
>>>>>>> latesttryupdate

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
<<<<<<< HEAD
<<<<<<< HEAD
	
	@Override
	public void robotInit() {
		m_oi = new OI();
		m_chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
=======
	@Override
	public void robotInit() {
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
	
	@Override
	public void robotInit() {
		oi = new OI();
		m_chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
>>>>>>> latesttryupdate
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
<<<<<<< HEAD
<<<<<<< HEAD
		m_autonomousCommand = m_chooser.getSelected();
=======
		autonomousCommand = chooser.getSelected();
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
		m_autonomousCommand = m_chooser.getSelected();
>>>>>>> latesttryupdate

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> latesttryupdate
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
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
<<<<<<< HEAD
<<<<<<< HEAD
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
=======
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
		driveSystem.reset();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		rightMaxV = 0;
		leftMaxV = 0;
	}
	double rightMaxV = 0;
	double leftMaxV = 0;
	/**
	 * This function is called periodically during operator control.
>>>>>>> latesttryupdate
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
<<<<<<< HEAD
	}

	/**
<<<<<<< HEAD
=======
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
<<<<<<< HEAD
=======
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
	}
	
	public void logToDashBoard()
	{
		SmartDashboard.putNumber("left raw distance", driveSystem.getLeftEncoder().getDistance());
		SmartDashboard.putNumber("right raw distance",driveSystem.getRightEncoder().getDistance());
		SmartDashboard.putNumber("left  XD velicity", driveSystem.getLeftEncoder().getRate());
		SmartDashboard.putNumber("right XD velocity", driveSystem.getRightEncoder().getRate());
		SmartDashboard.putNumber("left XD max v", leftMaxV);
		SmartDashboard.putNumber("right XD max v", rightMaxV);
<<<<<<< HEAD
>>>>>>> latesttryupdate
=======
		if(Robot.driveSystem.getLeftMPController().getSetpoint() != null)
			SmartDashboard.putNumber("velocity setpoint", Robot.driveSystem.getLeftMPController().getSetpoint().velocity);
//    	SmartDashboard.putNumber("position setpoint", setpoint.position);
>>>>>>> latesttryupdate
	}
}
