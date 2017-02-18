package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3459.robot.Cameras.CameraType;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * Don't change the name of this or it won't work. (The manifest looks for
 * "Robot")
 */
public class Robot extends IterativeRobot {
	public enum DriveType {
		ROBOT_RELATIVE_FRONT, ROBOT_RELATIVE_BACK, FIELD_RELATIVE
	};

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
	DriveType driveType = DriveType.ROBOT_RELATIVE_FRONT;
	// operations

	AHRS ahrs;
	// Joystick leftStick = new Joystick(RobotMap.leftStick);
	// Joystick rightStick = new Joystick(RobotMap.rightStick);
	Joystick driveStick = new Joystick(RobotMap.driveStick);
	ControlPanel controlPanel = new ControlPanel(RobotMap.controlPanel);

	public double speedInput(double input, boolean slow) {
		double output;
		if (slow)
			input = input / 2;
		output = input * input;
		if (input < 0.0)
			output = output * -1.0;
		return output;
	}

	public double normalizeAngle(double input) {
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
			ahrs = new AHRS(SerialPort.Port.kUSB);
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
		int pov = driveStick.getPOV();
		if (pov != -1) {
			double targetAngle = pov;
			if (pov > 180) {
				targetAngle = pov - 360;
			}
			driveTrain.turnToAngle(targetAngle);

		}
		if (driveStick.getRawButton(2)) {
			driveTrain.stopTurnToAngle();
		}
		if (driveStick.getRawButton(7))
			driveType = DriveType.ROBOT_RELATIVE_FRONT;
		if (driveStick.getRawButton(8))
			driveType = DriveType.FIELD_RELATIVE;

		double x = speedInput(driveStick.getX(), driveStick.getTrigger());
		double y = speedInput(driveStick.getY(), driveStick.getTrigger());
		double twist = speedInput(driveStick.getTwist(), driveStick.getTrigger());
		if (driveStick.getRawButton(4)) {
			driveTrain.drive(x, y, twist, 180.0);
			SmartDashboard.putString("driveMode", "robot relitve back");
		} else {

			switch (driveType) {
			case ROBOT_RELATIVE_FRONT:
				driveTrain.drive(x, y, twist, 0.0);
				SmartDashboard.putString("driveMode", "robot relitve front");
				break;

			case ROBOT_RELATIVE_BACK:
				driveTrain.drive(x, y, twist, 180.0);
				SmartDashboard.putString("driveMode", "robot relitve back");
				break;
			case FIELD_RELATIVE:
			default:
				driveTrain.drive(x, y, twist, normalizeAngle(ahrs.getAngle()));
				SmartDashboard.putString("driveMode", "Field relative ");
				break;

			}
		}
		SmartDashboard.putNumber("distance left", ultrasonicLeft.getDistance());
		SmartDashboard.putNumber("distance right", ultrasonicRight.getDistance());
		SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());
		SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
		SmartDashboard.putNumber("joystick", driveStick.getY());
		//testShooter();
		// testClimber();
		// testPickerUpper();
		if (driveStick.getRawButton(5)) {
			cameras.changeCamera(CameraType.FRONT);
		}
		if (driveStick.getRawButton(6)) {
			cameras.changeCamera(CameraType.BACK);
		}

	}

	public void testShooter() {
		if (driveStick.getRawButton(4))
			shooter.startWheels();
		if (driveStick.getRawButton(6))
			shooter.stopWheels();
		if (driveStick.getRawButton(11))
			shooter.startFeeder();
		if (driveStick.getRawButton(12))
			shooter.stopFeeder();
	}

	public void testClimber() {
		if (driveStick.getRawButton(10)) {
			climber.startClimber();

		} else {
			climber.stopClimber();
		}
	}

	public void testPickerUpper() {
		if (driveStick.getRawButton(5)) {
			pickerupper.startPickup();
		}
		if (driveStick.getRawButton(3)) {
			pickerupper.stopPickup();
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
		auto_driveForward();

	}

	public void auto_driveForward() {
		if (ultrasonicBack.getDistance() > 100.0) {
			driveTrain.drive(0.0, 0.5, 0.0, normalizeAngle(ahrs.getAngle()));// these
																				// numbers
																				// need
																				// changing
		} else {
			driveTrain.drive(0.0, 0.0, 0.0, 0.0);
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
