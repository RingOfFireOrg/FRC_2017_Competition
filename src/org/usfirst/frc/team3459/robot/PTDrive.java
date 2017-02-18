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
			mecanumDrive_Cartesian(x, y, kP * deltaAngle, currentAngle);
			if (Math.abs(deltaAngle) < 1.0) {
				stopTurnToAngle();
			}
		} else {
			mecanumDrive_Cartesian(x, y, twist, currentAngle);
		}
	}
}
