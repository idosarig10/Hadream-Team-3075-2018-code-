<<<<<<< HEAD
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
=======
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713

package org.usfirst.frc.team3075.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Thread.State;

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
import org.usfirst.frc.team3075.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
<<<<<<< HEAD
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	public static final ExampleSubsystem kExampleSubsystem
			= new ExampleSubsystem();
	public static OI m_oi;
	public static Chassis driveSystem = new Chassis();

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
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

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
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
	 * <p>You can add additional auto modes by adding additional commands to the
=======
	 * You can add additional auto modes by adding additional commands to the
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
<<<<<<< HEAD
		m_autonomousCommand = m_chooser.getSelected();
=======
		autonomousCommand = chooser.getSelected();
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
<<<<<<< HEAD
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
=======
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
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
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
<<<<<<< HEAD
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
=======
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	}
}
