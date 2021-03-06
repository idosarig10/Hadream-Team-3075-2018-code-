/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3075.robot;
 
import org.usfirst.frc.team3075.robot.commands.SetShift;
import org.usfirst.frc.team3075.robot.subsystems.Chassis;

import Autonomous.AutonomousLeftScale;
import Autonomous.AutonomousLeftSwitch;
import Autonomous.AutonomousMiddle;
import Autonomous.LeftScale;
import Autonomous.LeftSwitch;
import Autonomous.MiddlePickUp;
import Autonomous.MiddleSwitchLeft;
import LibPurple.sensors.ConsoleJoystick;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import edu.wpi.first.wpilibj.buttons.Button;

import org.usfirst.frc.team3075.robot.commands.ActiveIntake;
import org.usfirst.frc.team3075.robot.commands.AutoSetBigElevator;
import org.usfirst.frc.team3075.robot.commands.AutoSetElevators;
import org.usfirst.frc.team3075.robot.commands.AutoSetSmallElevator;
import org.usfirst.frc.team3075.robot.commands.ExampleCommand;
import org.usfirst.frc.team3075.robot.commands.FoldWheels;
import org.usfirst.frc.team3075.robot.commands.LockWheels;
import org.usfirst.frc.team3075.robot.commands.ReleaseCube;

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
	
	Button lowShiftButton = new JoystickButton(driverStick, 5);
	Button highShiftButton = new JoystickButton(driverStick, 6);
	Button driveforward = new JoystickButton(driverStick, 1);
	Button activeIntakeButton = new JoystickButton(driverStick, 3);
	Button releaseCubeButton = new JoystickButton(driverStick, 4);
	Button lockWheelsButton = new JoystickButton(driverStick, 8);
	Button flipDrive = new JoystickButton(driverStick, 7);
	Button foldWheels = new JoystickButton(elevatorStick, 2);
	Button autoSmallElevatorTop = new JoystickButton(elevatorStick, 4);
	Button autoSmallElevatorDown = new JoystickButton(elevatorStick, 1);
	Button autoBigElevatorTop = new JoystickButton(elevatorStick, 7);
	Button autoBigElevatorDown = new JoystickButton(elevatorStick, 8);
	Button autoSetElevators = new JoystickButton(elevatorStick, 3);

  	public OI() 
	{
		lowShiftButton.whenPressed(new SetShift(Chassis.Shift.Power));
		highShiftButton.whenPressed(new SetShift(Chassis.Shift.Speed));
		driveforward.whenPressed(new MiddleSwitchLeft());
//		driveforward.toggleWhenPressed(new );
		
		
//		driveforward.whenPressed(Robot.driveSystem.turnAngleTrapizodial(180));
//		driveforward.whenPressed(Robot.driveSystem.drive2D("/Paths/middle2_right.csv", "/Paths/middle2_left.csv", false));
//		driveforward.whenPressed(Robot.driveSystem.driveStraightTrapizodial(1, false));
		activeIntakeButton.toggleWhenPressed(new ActiveIntake(0.7, 0.7));
		lockWheelsButton.toggleWhenPressed(Robot.driveSystem.driveStraightTrapizodial(0, true));
		releaseCubeButton.whileHeld(new ReleaseCube());
		flipDrive.whenPressed(driverStick.reverseDirection(1));
		foldWheels.whenPressed(new FoldWheels());
		autoSmallElevatorTop.whenPressed(new AutoSetSmallElevator(Constants.smallElevatorTopPosition));
		autoSmallElevatorDown.whenPressed(new AutoSetSmallElevator(Constants.smallElevatorDownPosition));
		autoBigElevatorTop.toggleWhenPressed(new AutoSetBigElevator(Constants.bigElevatorTopPosition));
		autoBigElevatorDown.toggleWhenPressed(new AutoSetBigElevator(Constants.bigelevatorDownPosition));
		autoSetElevators.toggleWhenPressed(new AutoSetElevators(Constants.bigElevatorTopPosition, Constants.smallElevatorTopPosition));
		
//		driveforward.whenPressed(Robot.driveSystem.driveStraightTrapizodial(1, false));
//		driveforward.whenPressed(Robot.driveSystem.driveArc(1, 90, false));
	}
}
