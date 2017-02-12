package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PTDrive extends RobotDrive {
	boolean rotateToAngle = false;
	double targetAngle = 0.0f;
	double kP = 0.02f;

	public PTDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
            SpeedController frontRightMotor, SpeedController rearRightMotor) {
		super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		setInvertedMotor(MotorType.kFrontRight, true);
		setInvertedMotor(MotorType.kRearRight, true);

	}

	public static PTDrive buildDrive(final int frontLeftMotor, final int rearLeftMotor, final int frontRightMotor,
			final int rearRightMotor)	{
		SpeedController t_rearLeftMotor = createController(rearLeftMotor);
		SpeedController t_rearRightMotor = createController(rearRightMotor);
		SpeedController t_frontLeftMotor = createController(frontLeftMotor);
		SpeedController t_frontRightMotor = createController(frontRightMotor);
	    return new PTDrive(t_frontLeftMotor, t_rearLeftMotor, t_frontRightMotor, t_rearRightMotor);
	}

	private static SpeedController createController(final int motor) {
		DigitalInput input = new DigitalInput(9);
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
		double deltaAngle = targetAngle - currentAngle;
		if (deltaAngle > 180){
			deltaAngle = deltaAngle - 360;
		}
		if (deltaAngle < -180){
			deltaAngle = deltaAngle + 360;
		}
		return deltaAngle;
	}

	void drive(double x, double y, double twist, double currentAngle) {
		if (rotateToAngle) {
			double deltaAngle = getDeltaAngle(targetAngle, currentAngle);
			SmartDashboard.putNumber("targetAngle", targetAngle);
			SmartDashboard.putNumber("currentAngle", currentAngle);
			mecanumDrive_Cartesian(x, y, kP * deltaAngle, currentAngle);
			if (Math.abs(deltaAngle) < 1.0) {
				stopTurnToAngle();
			}
		} else {
			mecanumDrive_Cartesian(x, y, twist, currentAngle);
		}
	}
}
