package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.Joystick;

public class RobotMap {
	// motors
	public static final int frontLeftMotor = 0;
	public static final int frontRightMotor = 1;
	public static final int rearLeftMotor = 2;
	public static final int rearRightMotor = 3;
	// CAN INSTEAD OF PWM  // probably all of these numbers are not quite correct (CAN)
	public static final int shooterMotor = 4;
	public static final int pickupMotor = 5; 
	public static final int feederMotor = 9; //also feeder is a bad name, what does that mean?
	public static final int climberMotor1 = 7; // 
	public static final int climberMotor2 = 6; // 
	
	/*
	
	I have a Talon on ID 6 which should be for one of the climber motors, it has no code telling it to move
    I have a Talon on ID 7 which should be for one of the climber motors, it has no code telling it to move
	I have a Talon on ID 8 which should be for one of the hopper motors, it has no code telling it to move
	I have a Talon on ID 9 which should be for one of the hopper motors, it has no code telling it to move
	
	button four runs Talon ID 4, which is/was what the shooterMoter set to
	button six runs Talon ID 5, which is/was what the PickupMotor was set to
	
	
	*/
	
	// Joysticks
	// public static int leftStick = 1; not currently used
	// public static int rightStick = 2; not currently used
	public static final int driveStick = 0;
	public static final int xBoxController = 3;
	public static final int controlPanel = 1;
	public static final int turnToAngleController = 2;
	
	// axis numbers for xbox controller
	// xbc = xboxcontroller
	
	
	static class XBox {
	   public static final int leftStickX=1, leftStickY=2, rightStickX=4, rightStickY=5;

	   public static double getDirection(Joystick xBoxController){
			return Math.toDegrees(Math.atan2(xBoxController.getRawAxis(rightStickX), xBoxController.getRawAxis(rightStickY)));
		}
	}

	
	
	// analog ports
	public static final int ultrasonicBack = 0;
	public static final int ultrasonicLeft = 1;
	public static final int ultrasonicRight = 2;
	
	// digital io
	public static final int TestBotCartridge=9;


	// control panel
	public static final int selector_1 = 1;
	public static final int selector_2 = 2;
	public static final int selector_3 = 3;
	
	public static final int btnShooter=4;
	public static final int btnClimber=8;
	public static final int btnPickupper=6;
	public static final int btnEmergencyReverse=7; // btw this doesn't work in conjuction with running the btnShooter
	
}
