package org.usfirst.frc.team3459.robot;

import org.usfirst.frc.team3459.robot.Cameras.CameraType;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Don't change the name of this or it won't work. (The manifest looks for
 * "Robot")
 */
public class Robot extends IterativeRobot {

	/*
	 * Member variables go here
	 */
	UltrasonicSensor ultrasonicBack = new UltrasonicSensor(RobotMap.ultrasonicBack);
	UltrasonicSensor ultrasonicLeft = new UltrasonicSensor(RobotMap.ultrasonicLeft);
	UltrasonicSensor ultrasonicRight = new UltrasonicSensor(RobotMap.ultrasonicRight);
	PTDrive driveTrain;
	Shooter shooter = new Shooter();
	Climber climber = new Climber();
	PickerUpper pickerupper = new PickerUpper();
	Cameras cameras;
	PTDrive.DriveType driveType = PTDrive.DriveType.FIELD_RELATIVE;
	// operations

	AHRS ahrs;
	// Joystick leftStick = new Joystick(RobotMap.leftStick);
	// Joystick rightStick = new Joystick(RobotMap.rightStick);
	Joystick driveStick = new Joystick(RobotMap.driveStick);
	XBoxController xbc = new XBoxController(RobotMap.xBoxController);
	ControlPanel controlPanel = new ControlPanel(RobotMap.controlPanel);
	TurnToAngleController angleButtons = new TurnToAngleController(RobotMap.turnToAngleController);


	public double speedInput(double input, boolean slow) {
		double output;
		if (slow)
			input = input / 2;
		output = input * input;
		if (input < 0.0)
			output = output * -1.0;
		return output;
	}

	public static double normalizeAngle(double input) {
		double output = input;
		while (output > 180) {
			output = output - 360;
		}
		while (output < -180) {
			output = output + 360;
		}
		return output;
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		cameras = new Cameras();
		try {
			ahrs = new AHRS(I2C.Port.kOnboard);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		driveTrain = PTDrive.buildDrive(RobotMap.frontLeftMotor, RobotMap.rearLeftMotor, RobotMap.frontRightMotor,
				RobotMap.rearRightMotor);

	}

	/**
	 * This function is called once when we go into the teleop mode
	 */
	public void teleopInit() {
		ahrs.reset();
	}

	/**
	 * This function is called periodically during operator control (approx
	 * 20ms)
	 */
	public void teleopPeriodic() {
		int pov = angleButtons.getPOV();

		if (pov != -1) {
			driveTrain.turnToAngle(normalizeAngle(pov));
		}

		// testing stuff

		// driveTrain.turnToAngle(xbc.getDirection());

		// testing stuff

		if (driveStick.getRawButton(2)) {
			driveTrain.stopTurnToAngle();
		}
		if (driveStick.getRawButton(7))
			driveType = PTDrive.DriveType.ROBOT_RELATIVE_FRONT;
		if (driveStick.getRawButton(8))
			driveType = PTDrive.DriveType.FIELD_RELATIVE;

		if (driveStick.getRawButton(11))
			driveTrain.turnToAngle(30.0);

		double x = speedInput(driveStick.getX(), driveStick.getTrigger());
		double y = speedInput(driveStick.getY(), driveStick.getTrigger());
		double twist = speedInput(driveStick.getTwist(), driveStick.getTrigger());
		if (driveStick.getRawButton(4)) {
			driveTrain.drive(x, y, twist, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.ROBOT_RELATIVE_BACK);
			cameras.changeCamera(CameraType.BACK);
		} else {
			driveTrain.drive(x, y, twist, normalizeAngle(ahrs.getAngle()), driveType);
		}
		SmartDashboard.putNumber("distance left", ultrasonicLeft.getDistance());
		SmartDashboard.putNumber("distance right", ultrasonicRight.getDistance());
		SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());
		SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
		SmartDashboard.putNumber("rawangle", ahrs.getAngle());
		SmartDashboard.putNumber("targetangle", normalizeAngle(pov));

		if (driveStick.getRawButton(5)) {
			cameras.changeCamera(CameraType.FRONT);
		}
		if (driveStick.getRawButton(6)) {
			cameras.changeCamera(CameraType.BACK);
		}
		// manipulator
		if (controlPanel.getShooter()) {
			shooter.startFeeder();
			shooter.startWheels();
		} else {
			shooter.stopFeeder();
			shooter.stopWheels();
		}

		if (controlPanel.getClimber()) {
			if (controlPanel.getEmergencyReverse()) {
				climber.reverse();
			} else {
				climber.start();
			}
		} else {
			climber.stop();
		}
		if (controlPanel.getPickerUpper()) {
			if (controlPanel.getEmergencyReverse()) {
				pickerupper.reverse();
			} else {
				pickerupper.start();
			}
		} else {
			pickerupper.stop();
		}
	}

	/**
	 * This function is called once when we go into the Autonomous mode
	 */
	public void autonomousInit() {
		ahrs.reset();
		int selector = controlPanel.getProgram();
		SmartDashboard.putNumber("Autonomous Program", selector);
	}

	/**
	 * This function is called periodically during autonomous control (approx
	 * 20ms)
	 */
	public void autonomousPeriodic() {
		switch (controlPanel.getProgram()) {
		case 1:
			auto_driveForward();
			break;
		case 2:
			auto_depositGear();
			break;
		case 3:
			auto_depositGear3();
			break;
		case 4:
			auto_shoot();
			break;
		default:
			auto_driveForward();
			break;
		}
	}

	public void auto_depositGear() {
		// TODO
	}

	public void auto_depositGear3() {
		// TODO
	}

	public void auto_shoot() {
		// TODO
	}

	public void auto_driveForward() {
		if (ultrasonicBack.getDistance() > 100.0) {
			driveTrain.drive(0.0, 0.5, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
		} else {
			driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.ROBOT_RELATIVE_FRONT);
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run(); // This makes sure the values of items are correct on
							// the driver station during test mode.
	}
}
