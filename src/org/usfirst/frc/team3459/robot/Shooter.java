package org.usfirst.frc.team3459.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
	CANTalon wheelControl;
	CANTalon feederControl;
	CANTalon agitatorControl;

	public Shooter() {
		wheelControl = new CANTalon(RobotMap.shooterMotor);
		feederControl = new CANTalon(RobotMap.feederMotor);
		agitatorControl = new CANTalon(RobotMap.agitatorMotor);
	}

	public boolean startWheels() {
		double wheelSpeed = SmartDashboard.getNumber("Shooter Speed", -1.0);  //TODO: Needs to be changed 
		//A negative speed is required in the shooter to shoot balls normally
		SmartDashboard.putNumber("Shooter Speed", wheelSpeed);
		wheelControl.set(wheelSpeed); 
		return true;
	}
	

	public boolean stopWheels() {
		//System.out.println("hello");
		this.stopFeeder();
		wheelControl.set(0);
		return true;

	}

	public boolean startFeeder() {
		if (this.areWheelsOn()== false)
			this.startWheels();
		feederControl.set(1.0);// TODO DONT KNOW WHAT SPEED IS NEEDED
		agitatorControl.set(-1.0);
		return true;
	}

	public boolean stopFeeder() {
		feederControl.set(0);
		agitatorControl.set(0);
		return true;
	}

	public boolean isFeederOn() {
		if (feederControl.get() != 0.0)
			return true;
		return false;
	}

	public boolean areWheelsOn() {
		if (wheelControl.get() != 0.0)
			return true;
		return false;
	}
}
