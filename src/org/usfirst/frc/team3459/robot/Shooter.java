package org.usfirst.frc.team3459.robot;

import com.ctre.CANTalon;

public class Shooter {
	CANTalon wheelControl;
	CANTalon feederControl;

	public Shooter() {
		wheelControl = new CANTalon(RobotMap.shooterMotor);
		feederControl = new CANTalon(RobotMap.feederMotor);
	}

	public boolean startWheels() {
		wheelControl.set(1.0); // TODO DONT KNOW WHAT SPEED IS NEEDED
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
		return true;
	}

	public boolean stopFeeder() {
		feederControl.set(0);
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
