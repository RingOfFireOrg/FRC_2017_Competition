package org.usfirst.frc.team3459.robot;

import org.usfirst.frc.team3459.robot.Cameras.CameraType;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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
	int autoStep = 1;
	Timer autoTimer;
	// operations

	AHRS ahrs;
	// Joystick leftStick = new Joystick(RobotMap.leftStick);
	// Joystick rightStick = new Joystick(RobotMap.rightStick);
	Joystick driveStick = new Joystick(RobotMap.driveStick);
	LogitechController ltc = new LogitechController(5);
	// XBoxController xbc = new XBoxController(RobotMap.xBoxController);
	ControlPanel controlPanel = new ControlPanel(RobotMap.controlPanel);
	// TurnToAngleController angleButtons = new
	// TurnToAngleController(RobotMap.turnToAngleController);

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
		ahrs.reset();
		autoTimer = new Timer();
	}

	/**
	 * This function is called once when we go into the teleop mode
	 */
	public void teleopInit() {
		// ahrs.reset();
	}

	/**
	 * This function is called periodically during operator control (approx
	 * 20ms)
	 */
	public void teleopPeriodic() {
		int pov = driveStick.getPOV();

		if (pov != -1) {
			driveTrain.turnToAngle(normalizeAngle(pov));
		}

		if (driveStick.getRawButton(RobotMap.btnStopTurn)) {
			driveTrain.stopTurnToAngle();
		}
		if (driveStick.getRawButton(RobotMap.btnRobotRel))
			driveType = PTDrive.DriveType.ROBOT_RELATIVE_FRONT;
		if (driveStick.getRawButton(RobotMap.btnFieldRel))
			driveType = PTDrive.DriveType.FIELD_RELATIVE;

		if (driveStick.getRawButton(RobotMap.btnTurnRight))
			driveTrain.turnToAngle(30.0);
		// driveTrain.mecanumDrive_Cartesian(0.4, 0.0, 0, ROBOT_RELATIVE_FRONT);

		if (driveStick.getRawButton(RobotMap.btnTurnLeft))
			driveTrain.turnToAngle(-30.0);
		// driveTrain.mecanumDrive_Cartesian(-0.4, 0.0, 0.0, 0.0);

		if (ltc.getRawButton(1)) {
			ahrs.reset();
			ahrs.setAngleAdjustment(180.0);
		}

		double x, y, twist;
		if (SmartDashboard.getBoolean("this is a logitech controller", true)) {
			if (Math.abs(ltc.getLeftX()) > 0.3) {
				x = ltc.getLeftX() / 2;
			} else {
				x = 0;
			}

			if (Math.abs(ltc.getLeftY()) > 0.3) {
				y = ltc.getLeftY() / 2;
			} else {
				y = 0;
			}

			if (Math.abs(ltc.getRightX()) > 0.1 || Math.abs(ltc.getRightY()) > 0.1) {
				twist = PTDrive.getSpeed(PTDrive.getDeltaAngle(ltc.getDirection() + 180 , ahrs.getAngle()));
			} else {
				twist = 0;
			}

		} else {

			x = speedInput(driveStick.getX(), driveStick.getTrigger());
			y = speedInput(driveStick.getY(), driveStick.getTrigger());
			twist = speedInput(driveStick.getTwist(), driveStick.getTrigger());
		}

		if (driveStick.getRawButton(RobotMap.btnRobotRelBack)) {
			driveTrain.drive(x, y, twist, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.ROBOT_RELATIVE_BACK);
		} else {
			driveTrain.drive(x, y, twist, normalizeAngle(ahrs.getAngle()), driveType);
		}
		SmartDashboard.putNumber("distance left", ultrasonicLeft.getDistance());
		SmartDashboard.putNumber("distance right", ultrasonicRight.getDistance());
		SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());
		SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
		SmartDashboard.putNumber("rawangle", ahrs.getAngle());
		SmartDashboard.putNumber("targetangle", normalizeAngle(pov));

		if (driveStick.getRawButton(RobotMap.btnCameraFront)) {
			cameras.changeCamera(CameraType.FRONT);

		}
		if (driveStick.getRawButton(RobotMap.btnCameraBack)) {
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
		autoStep = 1;
		autoTimer.reset();
		autoTimer.start();
	}

	/**
	 * This function is called periodically during autonomous control (approx
	 * 20ms)
	 */
	public void autonomousPeriodic() {
		DriverStation.Alliance alliance = DriverStation.getInstance().getAlliance();
		switch (controlPanel.getProgram()) {

		case RobotMap.cardBlue:
			auto_driveForward();
			break;
		case RobotMap.cardRed:
			auto_depositGear();
			break;
		case RobotMap.cardBlack:
			auto_depositGear3(alliance);
			break;
		// case RobotMap.cardGreen:
		// auto_shoot();
		// break;
		default:
			break;
		}
	}

	public void auto_depositGear() {
		if (autoTimer.get() < 4.0) {
			driveTrain.drive(0.0, 0.4, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
		} else {
			driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);

		}

	}

	public void auto_depositGear3(DriverStation.Alliance myAlliance) {
		final int kDriveDistance = 100; // TODO: We know this is wrong

		switch (myAlliance) {
		case Red:
			switch (autoStep) {
			case 1:
				if (ultrasonicBack.getDistance() < kDriveDistance) {
					driveTrain.drive(0.0, 0.5, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
				} else {
					driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
					autoStep = 2;
				}
				break;
			case 2:
				// if (ultrasonicRight.getDistance() )
				break;
			case 3:
				if (normalizeAngle(ahrs.getAngle()) < 30) {
					driveTrain.drive(0.0, 0.0, 0.5, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
				} else {
					driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
					autoStep = 4;
					autoTimer.reset();
					autoTimer.start();
				}
				break;
			case 4:
				if (autoTimer.get() < 5.0) {
					driveTrain.drive(0.0, 0.5, 0.0, normalizeAngle(ahrs.getAngle()),
							PTDrive.DriveType.ROBOT_RELATIVE_BACK);
				} else {
					driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
					autoStep = 5;
					autoTimer.reset();
					autoTimer.start();
				}
				break;
			case 5:
				if (autoTimer.get() < 0.3) {
					driveTrain.drive(0.0, 0.5, 0.0, normalizeAngle(ahrs.getAngle()),
							PTDrive.DriveType.ROBOT_RELATIVE_FRONT);
				} else {
					driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
					autoStep = 6;
				}
				break;
			case 6:
				if (normalizeAngle(ahrs.getAngle()) > 0) {
					driveTrain.drive(0.0, 0.0, 0.5, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
				} else {
					driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
					autoStep = 7;
					autoTimer.reset();
					autoTimer.start();
				}
				break;
			case 7:
				if (autoTimer.get() < 2.0) {
					driveTrain.drive(0.0, 0.5, 0.0, normalizeAngle(ahrs.getAngle()),
							PTDrive.DriveType.ROBOT_RELATIVE_BACK);
				} else {
					driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
					autoStep = 8;

				}
				break;
			}

		case Blue:
			break;
		case Invalid:
			break;
		}
	}

	// TODO

	public void auto_shoot() {
		// TODO
	}

	public void auto_driveForward() {
		if (ultrasonicBack.getDistance() < 100.0) {
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
