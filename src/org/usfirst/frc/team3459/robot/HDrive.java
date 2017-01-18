package org.usfirst.frc.team3459.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;

public class HDrive extends RobotDrive {

	CANTalon shimmyMotor1 = new CANTalon(RobotMap.shimmyTalon1); 
	CANTalon shimmyMotor2 = new CANTalon(RobotMap.shimmyTalon2); 
	
	public HDrive(int leftMotor, int rightMotor, int shimmyMotor){
		super(leftMotor, rightMotor);
	   
	}
	private void shimmy (double speed) {
		shimmyMotor1.set(speed);
		shimmyMotor2.set(speed);
	}
	public void shimmyLeft(){
		shimmy(0.75);
	}
	public void shimmyRight(){
		shimmy(-0.75);
	}
	public void shimmyStop(){
		shimmy(0.0);
	}
}
