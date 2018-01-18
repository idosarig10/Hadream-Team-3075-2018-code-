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

package org.usfirst.frc.team3075.robot;

import org.usfirst.frc.team3075.robot.commands.SetShift;
import org.usfirst.frc.team3075.robot.subsystems.Chassis;

import LibPurple.sensors.ConsoleJoystick;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
<<<<<<< HEAD
=======
package org.usfirst.frc.team3075.robot;

import edu.wpi.first.wpilibj.buttons.Button;

import org.usfirst.frc.team3075.robot.commands.ExampleCommand;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
>>>>>>> latesttryupdate

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> latesttryupdate
	public ConsoleJoystick xbox = new ConsoleJoystick(0);

	Button lowShift = new JoystickButton(xbox, 5);
	Button highShift = new JoystickButton(xbox, 6);
<<<<<<< HEAD

=======
	Button driveforward = new JoystickButton(xbox, 1);
=======
	public ConsoleJoystick driverStick = new ConsoleJoystick(0);
	public ConsoleJoystick elevatorStick = new ConsoleJoystick(1);
	
	Button lowShift = new JoystickButton(driverStick, 5);
	Button highShift = new JoystickButton(driverStick, 6);
	Button driveforward = new JoystickButton(driverStick, 1);
>>>>>>> latesttryupdate
	
>>>>>>> latesttryupdate
	public OI() 
	{
		lowShift.whenPressed(new SetShift(Chassis.Shift.Power));
		highShift.whenPressed(new SetShift(Chassis.Shift.Speed));
<<<<<<< HEAD
<<<<<<< HEAD
	}
=======
	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
//		driveforward.whenPressed(Robot.driveSystem.driveStraightTrapizodial(1, false));
		driveforward.whenPressed(Robot.driveSystem.driveCurve(1, 90, false));
=======
//		driveforward.whenPressed(Robot.driveSystem.driveStraightTrapizodial(2, false));
		driveforward.whenPressed(Robot.driveSystem.driveArc(1, 90, false));
>>>>>>> latesttryupdate
	}
>>>>>>> latesttryupdate
}
