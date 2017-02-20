package org.usfirst.frc.team3459.robot;

public class RobotMap {
	// motors
	public static final int frontLeftMotor = 0;
	public static final int frontRightMotor = 1;
	public static final int rearLeftMotor = 2;
	public static final int rearRightMotor = 3;
	// CAN INSTEAD OF PWM
	public static final int shooterMotor = 14;
	public static final int feederMotor = 1;
	public static final int climberMotor = 3;
	public static final int pickupMotor = 2;

	// Joysticks
	// public static int leftStick = 1; not currently used
	// public static int rightStick = 2; not currently used
	public static final int driveStick = 0;
	public static final int controlPanel = 1;
	public static final int turnToAngleController = 2;

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
	public static final int btnEmergencyReverse=7;
	
}
