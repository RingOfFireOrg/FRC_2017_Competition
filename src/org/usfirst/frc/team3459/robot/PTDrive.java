package org.usfirst.frc.team3459.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PTDrive extends RobotDrive {
	public enum DriveType {
		ROBOT_RELATIVE_FRONT, ROBOT_RELATIVE_BACK, FIELD_RELATIVE
	};

	boolean rotateToAngle = false;
	double targetAngle = 0.0f;
	double kP = 0.02f;

	public PTDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor,
			SpeedController rearRightMotor) {
		super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		setInvertedMotor(MotorType.kFrontRight, true);
		setInvertedMotor(MotorType.kRearRight, true);

	}

	public static PTDrive buildDrive(final int frontLeftMotor, final int rearLeftMotor, final int frontRightMotor,
			final int rearRightMotor) {
		DigitalInput input = new DigitalInput(RobotMap.TestBotCartridge);

		SpeedController t_rearLeftMotor = createController(rearLeftMotor, input);
		SpeedController t_rearRightMotor = createController(rearRightMotor, input);
		SpeedController t_frontLeftMotor = createController(frontLeftMotor, input);
		SpeedController t_frontRightMotor = createController(frontRightMotor, input);
		SmartDashboard.putString("Motor Type", t_rearLeftMotor.getClass().getCanonicalName());
		return new PTDrive(t_frontLeftMotor, t_rearLeftMotor, t_frontRightMotor, t_rearRightMotor);
	}

	public static SpeedController createController(final int motor, DigitalInput input) {
		if (input.get()) {
			return new Victor(motor);
		} else {
			return new Talon(motor);
		}

	}

	public void turnToAngle(double angle) {
		rotateToAngle = true;
		targetAngle = angle;
	}

	public void stopTurnToAngle() {
		rotateToAngle = false;
	}

	private double getDeltaAngle(double targetAngle, double currentAngle) {
		return Robot.normalizeAngle(targetAngle - currentAngle);
	}

	private double getSpeed(double deltaAngle) {
		double scale = 0.3;
		double offset = 0.2;
		if (deltaAngle < 0.0){
			offset = offset * -1;
		}
		return (((deltaAngle / 180.0) * scale) + offset);
	}

	private void internalDrive(double x, double y, double twist, double currentAngle, DriveType driveType) {
		switch (driveType) {
		case ROBOT_RELATIVE_FRONT:
			mecanumDrive_Cartesian(x, y, twist, 180);
			SmartDashboard.putString("driveMode", "robot relative front");
			break;

		case ROBOT_RELATIVE_BACK:	
			mecanumDrive_Cartesian(x, y, twist, 0.0);
			SmartDashboard.putString("driveMode", "robot relative back");
			break;
		case FIELD_RELATIVE:
		default:
			mecanumDrive_Cartesian(x, y, twist, currentAngle);
			SmartDashboard.putString("driveMode", "Field relative ");
			break;

		}
	}

	XBoxController xbc = new XBoxController(RobotMap.xBoxController);
	AHRS ahrs;
	LogitechController ltc = new LogitechController(5);
	Joystick driveStick = new Joystick(RobotMap.driveStick);

	public void drive(double x, double y, double twist, double currentAngle, DriveType driveType) {
		if (rotateToAngle) {

			double targetAngle = driveStick.getDirectionDegrees();
			currentAngle = Robot.normalizeAngle(ahrs.getAngle());
			double deltaAngle = getDeltaAngle(targetAngle, currentAngle);
			
			
			internalDrive(x, y, getSpeed(deltaAngle), currentAngle, driveType);

		}

		else {
			internalDrive(x, y, twist, currentAngle, driveType);
		}

	}
}
