package org.usfirst.frc.team3459.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	// boolean isJogging = false;
	double direction = 0.0;

	SendableChooser<Integer> autoChooser;
	// operations

	AHRS ahrs;

	LogitechController ltc = new LogitechController(5);
	ControlPanel controlPanel = new ControlPanel(RobotMap.controlPanel);

	public double speedInput(double input, boolean slow) {
		double output;

		output = input * input;
		if (input < 0.0)
			output = output * -1.0;
		if (slow)
			output = output * 0.65;
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
		autoChooser = new SendableChooser<Integer>();
		autoChooser.addObject("GearBoiler", RobotMap.autoGearBoiler);
		autoChooser.addDefault("GearStraight", RobotMap.autoGearStraight);
		autoChooser.addObject("GearFeederStation", RobotMap.autoGearFeederStation);
		SmartDashboard.putData("Auto", autoChooser);

	}

	/**
	 * This function is called once when we go into the teleop mode
	 */
	public void teleopInit() {
		// ahrs.reset();
		SmartDashboard.putBoolean("is this a logitech controller", true);
	}

	/**
	 * This function is called periodically during operator control (approx
	 * 20ms)
	 */
	public void teleopPeriodic() {

		if (ltc.getRawButton(RobotMap.ltcBackButton)) {
			ahrs.reset();
			ahrs.setAngleAdjustment(0.0);
		}

		double x, y, twist;

		x = -1 * speedInput(ltc.getLeftX(), ltc.getTriggers()); // negative
																// so that
																// gear
																// holder is
																// forward.

		y = -1 * speedInput(ltc.getLeftY(), ltc.getTriggers()); // negative
		// so that
		// gear
		// holder is
		// forward.
		twist = 0;

		if (Math.abs(ltc.getRightX()) > 0.1 || Math.abs(ltc.getRightY()) > 0.1) {
			double deltaAngle = PTDrive.getDeltaAngle(ltc.getDirection(), ahrs.getAngle());
			if (Math.abs(deltaAngle) > 0.5) {
				twist = 1.3 * PTDrive.getSpeed(deltaAngle);
			}
		}

		driveTrain.drive(x, y, twist, normalizeAngle(ahrs.getAngle()), driveType);

		SmartDashboard.putNumber("distance left", ultrasonicLeft.getDistance());
		SmartDashboard.putNumber("distance right", ultrasonicRight.getDistance());
		SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());
		SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
		SmartDashboard.putNumber("rawangle", ahrs.getAngle());

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
		int selector = autoChooser.getSelected();

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
		SmartDashboard.putNumber("auto timer", autoTimer.get());
		SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());
		DriverStation.Alliance alliance = DriverStation.getInstance().getAlliance();
		switch (autoChooser.getSelected()) {

		case RobotMap.autoGearStraight:
			auto_depositGear();
			break;

		case RobotMap.autoGearBoiler:
			auto_depositGearBoiler(alliance);
			break;

		case RobotMap.autoGearFeederStation:
			auto_depositGearFeederStationSide(alliance);
			break;

		default:
			break;
		}
	}

	public void auto_depositGear() {
		if (autoTimer.get() < 4.0) {

			driveTrain.drive(0.0, 0.25, 0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(0, ahrs.getAngle())),
					normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			// driveTrain.drive(0.0, 0.4, 0.0, normalizeAngle(ahrs.getAngle()),
			// PTDrive.DriveType.FIELD_RELATIVE);
		} else {
			driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);

		}

	}

	int targetAngle = 0;

	public void auto_depositGearFeederStationSide(DriverStation.Alliance myAlliance) {
		switch (autoStep) {
		case 1:
			// drive to get away from the wall a few inches
			SmartDashboard.putNumber("testingauto", ahrs.getAngle());// a test
																		// to
																		// make
																		// sure
																		// the
																		// selection
																		// processs
																		// is
																		// working
			if (ultrasonicBack.getDistance() < 15) { // may need to + or -
														// distance
				driveTrain.drive(0.0, 0.25, 0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(0, ahrs.getAngle())),

						normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			} else {
				autoStep = 2;
			}
			break;
		case 2:
			SmartDashboard.putNumber("testingauto2", ahrs.getAngle());// a test
																		// to
																		// make
																		// sure
																		// the
																		// selection
																		// process
																		// is
																		// working

			SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
			if (myAlliance == DriverStation.Alliance.Red) {
				targetAngle = -30;
			} else {
				targetAngle = 30;
			}
			// turn to 60 degrees(parallel to line) so we don't hit the airship
			if (Math.abs(normalizeAngle(ahrs.getAngle() - targetAngle)) < 1) { // bigger
																				// tolerance?
				autoStep = 4;
				autoTimer.reset();
			} else {
				// driveTrain.turnToAngle(normalizeAngle(targetAngle));
				driveTrain.drive(0.0, 0.0, PTDrive.getSpeed(PTDrive.getDeltaAngle(targetAngle, ahrs.getAngle())),
						normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			}

			break;
		case 4:
			SmartDashboard.putNumber("testingauto4", ahrs.getAngle());// a test
																		// to
																		// make
																		// sure
																		// the
																		// selection
																		// process
																		// is
																		// working
			// drive x" at .5 speed: get close
			// total inches we want to drive this direction is 71.37
			// changed to 51.37 because subtracting half of the bot length bc
			// pivoting at middle of bot
			SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());

			if (ultrasonicBack.getDistance() < 60) {// might have changed this
													// number wrong...

				driveTrain.drive(0.0, -0.35,
						0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(targetAngle, ahrs.getAngle())), 0.0,
						PTDrive.DriveType.ROBOT_RELATIVE_FRONT);

				/*
				 * driveTrain.drive(Math.cos(targetAngle * 2 * Math.PI / 360) *
				 * .5, Math.sin(targetAngle * 2 * Math.PI / 360) * .5, 0.5 *
				 * PTDrive.getSpeed(PTDrive.getDeltaAngle(targetAngle,
				 * ahrs.getAngle())), normalizeAngle(ahrs.getAngle()),
				 * PTDrive.DriveType.FIELD_RELATIVE);
				 */
			} else {
				autoStep = 5;
			}
			break;
		case 5:
			SmartDashboard.putNumber("testingauto5", ahrs.getAngle());// a test
																		// to
																		// make
																		// sure
																		// the
																		// selection
																		// process
																		// is
																		// working
			// drive the rest of the distance at .2 speed: be accurate
			SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());
			if (ultrasonicBack.getDistance() < 85) {// 85 was off that was the
													// last tested
				driveTrain.drive(0.0, -0.2, 0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(targetAngle, ahrs.getAngle())),
						0.0, PTDrive.DriveType.ROBOT_RELATIVE_FRONT);

			} else {
				autoStep = 6;
			}
			break;
		case 6:
			SmartDashboard.putNumber("testingauto6", ahrs.getAngle());// a test
																		// to
																		// make
																		// sure
																		// the
																		// selection
																		// process
																		// is
																		// working
			// turn to -30 degrees to face spring
			SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
			if (myAlliance == DriverStation.Alliance.Red) {
				targetAngle = 60;
			} else {
				targetAngle = -60;
			} // not sure this is right might need to be reversed
			driveTrain.turnToAngle(normalizeAngle(targetAngle));
			if (Math.abs(normalizeAngle(ahrs.getAngle() - targetAngle)) < 1) {
				autoStep = 7;
				autoTimer.reset();
			} else {
				driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			}
			break;
		case 7:
			// drive to the spring for x seconds
			// this goes the wrong direction exactly 180 off
			/*
			 * TODO lower speed of last part
			 */
		
			/*
			 * driveTrain.drive(0.0, -0.35,
						0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(targetAngle, ahrs.getAngle())), 0.0,
						PTDrive.DriveType.ROBOT_RELATIVE_FRONT);
			 */
			if (autoTimer.get() < 3) {
				driveTrain.drive(-Math.signum(targetAngle) * Math.cos(-targetAngle * Math.PI / 360) * .25,
						Math.sin(Math.abs(targetAngle) * Math.PI / 360) * .25,
						0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(targetAngle, ahrs.getAngle())),
						normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			} else {
				driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			}
		}
	}

	public void auto_depositGearBoiler(DriverStation.Alliance myAlliance) {
		switch (autoStep) {
		case 1:
			SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());
			//shooter.startWheels();
			if (ultrasonicBack.getDistance() < 40) {
				driveTrain.drive(0.0, 0.5, 0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(0, ahrs.getAngle())),
						normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			} else {
				autoStep = 2;
				
			}
			break;
		case 2:
			SmartDashboard.putNumber("distance back", ultrasonicBack.getDistance());
			// Original number is 59.25 based on the math
			if (ultrasonicBack.getDistance() < 58.25) {
				driveTrain.drive(0.0, 0.2, 0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(0, ahrs.getAngle())),
						normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			} else {
				//shooter.startFeeder();//feeder is agitator and flower
				autoStep = 3;
			}
			break;
		case 3:
			SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
			SmartDashboard.putNumber("autoTargetAnglecase3", targetAngle);
			if (myAlliance == DriverStation.Alliance.Red) {
				targetAngle = -60;// why is this 60??? shouldn't it be 30???
			} else {
				targetAngle = 60;
			}
			driveTrain.turnToAngle(normalizeAngle(targetAngle));
			if (Math.abs(normalizeAngle(ahrs.getAngle() - targetAngle)) < 1) {
				autoStep = 4;
				autoTimer.reset();
			} else {
				driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			}
			break;
	
		case 4:
			SmartDashboard.putNumber("angle", normalizeAngle(ahrs.getAngle()));
			if (autoTimer.get() < 3) {
				// might need to change target angle back to 30
				driveTrain.drive(-Math.signum(targetAngle) * Math.cos(-targetAngle * Math.PI / 360) * .25,
						Math.sin(Math.abs(targetAngle) * Math.PI / 360) * .25,
						0.5 * PTDrive.getSpeed(PTDrive.getDeltaAngle(targetAngle, ahrs.getAngle())),
						normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			} else {
				driveTrain.drive(0.0, 0.0, 0.0, normalizeAngle(ahrs.getAngle()), PTDrive.DriveType.FIELD_RELATIVE);
			}
		}
	}

	public void testInit() {

	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run(); // This makes sure the values of items are correct on
							// the driver station during test mode.

		if (ltc.getRawButton(RobotMap.ltcXButton)) {
			driveTrain.test(true, true, 1.0);
		} else {
			driveTrain.test(true, true, 0.0);
		}

		if (ltc.getRawButton(RobotMap.ltcYButton)) {
			driveTrain.test(true, false, 1.0);
		} else {
			driveTrain.test(true, false, 0.0);
		}

		if (ltc.getRawButton(RobotMap.ltcAButton)) {
			driveTrain.test(false, true, 1.0);
		} else {
			driveTrain.test(false, true, 0.0);
		}

		if (ltc.getRawButton(RobotMap.ltcBButton)) {
			driveTrain.test(false, false, 1.0);
		} else {
			driveTrain.test(false, false, 0.0);
		}
	}
}