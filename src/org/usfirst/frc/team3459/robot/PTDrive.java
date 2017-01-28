package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;


public class PTDrive extends RobotDrive {
	boolean rotateToAngle = false;
	double targetAngle = 0.0f;
	double kP = 0.03f;
	public PTDrive(int frontLeftMotor, int rearLeftMotor, int frontRightMotor, int rearRightMotor){
		super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);	 
		setInvertedMotor(MotorType.kFrontRight, true);
		setInvertedMotor(MotorType.kRearRight, true);
		
	}
	public void turnToAngle(double angle){
	   rotateToAngle = true;	
	   targetAngle = angle;
	}
	public void stopTurnToAngle(){
	   rotateToAngle = false;
	}
	void drive(double x, double y, double twist, double currentAngle){
	if (rotateToAngle) {
		currentAngle = currentAngle % 180;
		double deltaAngle = targetAngle - currentAngle;
		mecanumDrive_Cartesian(x, y, kP * deltaAngle, currentAngle); 
	if (Math.abs(deltaAngle) < 1.0){
		stopTurnToAngle();
			
	}
	}
	else {
		mecanumDrive_Cartesian(x, y, twist, currentAngle); 
	}
	}
}
