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

import edu.wpi.first.wpilibj.buttons.Button;

import org.usfirst.frc.team3075.robot.commands.ExampleCommand;

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

	public ConsoleJoystick driverStick = new ConsoleJoystick(0);
	public ConsoleJoystick elevatorStick = new ConsoleJoystick(1);
	
	Button lowShift = new JoystickButton(driverStick, 5);
	Button highShift = new JoystickButton(driverStick, 6);
	Button driveforward = new JoystickButton(driverStick, 1);

	public OI() 
	{
		lowShift.whenPressed(new SetShift(Chassis.Shift.Power));
		highShift.whenPressed(new SetShift(Chassis.Shift.Speed));
		driveforward.whenPressed(Robot.driveSystem.driveStraightTrapizodial(1, false));
//		driveforward.whenPressed(Robot.driveSystem.driveStraightTrapizodial(2, false));
//		driveforward.whenPressed(Robot.driveSystem.driveArc(1, 90, false));
	}
}
