package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * Don't change the name of this or it won't work. (The manifest looks for
 * "Robot")
 */
public class Robot extends IterativeRobot {

	/*
	 * Member variables go here
	 */
	UltrasonicSensor rangeFinder = new UltrasonicSensor();
	PTDrive driveTrain = new PTDrive(RobotMap.frontLeftMotor, RobotMap.rearLeftMotor, RobotMap.frontRightMotor,
			RobotMap.rearRightMotor);
	Shooter shooter = new Shooter();

	// operations

	AHRS ahrs;
	// Joystick leftStick = new Joystick(RobotMap.leftStick);
	// Joystick rightStick = new Joystick(RobotMap.rightStick);
	Joystick driveStick = new Joystick(RobotMap.driveStick);
	ControlPanel controlPanel = new ControlPanel(RobotMap.controlPanel);

	public double speedInput(double input) {
		double output;

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
		try {
			ahrs = new AHRS(SerialPort.Port.kUSB);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
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
		driveTrain.drive(speedInput(driveStick.getX()), speedInput(driveStick.getY()),
				speedInput(driveStick.getTwist()), normalizeAngle(ahrs.getAngle()));
		SmartDashboard.putNumber("distance", rangeFinder.getDistance());
		SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
		this.testShooter();
	}

	public void testShooter() {
		if (driveStick.getRawButton(7))
			shooter.startWheels();
		if (driveStick.getRawButton(8))
			shooter.stopWheels();
		if (driveStick.getRawButton(11))
			shooter.startFeeder();
		if (driveStick.getRawButton(12))
			shooter.stopFeeder();
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
		driveTrain.turnToAngle(90.0f);
		driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()));
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run(); // This makes sure the values of items are correct on
							// the driver station during test mode.
	}
}
