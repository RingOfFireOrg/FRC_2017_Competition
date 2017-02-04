package org.usfirst.frc.team3459.robot;

import com.ctre.CANTalon;

public class PickerUpper {
	CANTalon pickupControl;
	
	public PickerUpper() {
		pickupControl = new CANTalon(RobotMap.pickupMotor);
	}

	public boolean startPickup() {
		pickupControl.set(1.0); // TODO DONT KNOW WHAT SPEED IS NEEDED
		return true;
	}
	public boolean stopPickup(){
		pickupControl.set(0.0);
		return true;
	}
}
