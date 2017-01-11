package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;

public class HDrive extends RobotDrive {
	Victor motor;
	public HDrive(int leftMotor, int rightMotor, int shimmyMotor){
		super(leftMotor, rightMotor);
	   motor = new Victor(shimmyMotor);
	}
	public void shimmyLeft(){
		motor.set(-1.0);
	}
	public void shimmyRight(){
		motor.set(1.0);
	}
	public void shimmyStop(){
		motor.set(0.0);
	}
}
