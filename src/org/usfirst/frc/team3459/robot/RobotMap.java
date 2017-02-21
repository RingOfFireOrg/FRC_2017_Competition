package org.usfirst.frc.team3459.robot;

public class RobotMap {
	// motors
	public static final int frontLeftMotor = 0;
	public static final int frontRightMotor = 1;
	public static final int rearLeftMotor = 2;
	public static final int rearRightMotor = 3;
	// CAN INSTEAD OF PWM
	public static final int shooterMotor = 4;
	public static final int pickupMotor = 5;
	public static final int feederMotor = 6; //also feeder is a bad name, what does that mean?
	public static final int climberMotor = 7; // this number is wrong, climber might run on two diff motor signals
	
// there are 6 motors  on can, on the robot I have hopper motors on ID 7 and 8
	// there are 4 other motor controllers, 
	
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
