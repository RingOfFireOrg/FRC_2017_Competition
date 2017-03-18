package org.usfirst.frc.team3459.robot;


public class RobotMap {
	// motors
	public static final int frontLeftMotor = 0;
	public static final int frontRightMotor = 1;
	public static final int rearLeftMotor = 2;
	public static final int rearRightMotor = 3;
	// CAN INSTEAD OF PWM  // probably all of these numbers are not quite correct (CAN)
	public static final int shooterMotor = 4;
	public static final int pickupMotor = 5; 
	public static final int feederMotor = 8; //also feeder is a bad name, what does that mean?
	public static final int climberMotor1 = 7; // 
	public static final int climberMotor2 = 6; // 
	public static final int agitatorMotor = 9;
	
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
	// driver buttons
	public static final int btnStopTurn = 2;
	public static final int btnRobotRel = 7;
	public static final int btnFieldRel = 8;
	public static final int btnRobotRelBack = 4;
	public static final int btnCameraFront = 5;
	public static final int btnCameraBack = 6;
	public static final int btnTurnLeft = 11;
	public static final int btnTurnRight = 12;
	public static final int btnResetGyro = 9;
	// logitech controller buttons
	public static final int leftTrigger = 7;
	public static final int rightTrigger = 8;
	public static final int ltcXButton = 1;
	public static final int ltcAButton = 2;
	public static final int ltcBButton = 3;
	public static final int ltcYButton = 4;
	public static final int ltcBackButton = 9;
	public static final int ltcStartButton = 10;
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
	public static final int cardBlue = 1;
	public static final int cardRed = 2;
	public static final int cardBlack = 3;
	public static final int cardGreen = 4;
	public static final int cardYellow = 5;
	
	// autonomous
	public static final int autoGearBoiler = 1;
	public static final int autoGearStraight = 2;
	public static final int autoGearFeederStation = 3;
	
	
	public static final int btnShooter=4;
	public static final int btnClimber=8;
	public static final int btnPickupper=6;
	public static final int btnEmergencyReverse=7; // btw this doesn't work in conjuction with running the btnShooter
	
}
