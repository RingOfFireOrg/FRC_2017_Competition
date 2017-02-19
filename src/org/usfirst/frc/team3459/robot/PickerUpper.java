package org.usfirst.frc.team3459.robot;

import com.ctre.CANTalon;

public class PickerUpper {
	CANTalon pickupControl;

	public PickerUpper() {
		pickupControl = new CANTalon(RobotMap.pickupMotor);
	}

	public boolean start() {
		pickupControl.set(1.0); // TODO DONT KNOW WHAT SPEED IS NEEDED
		return true;
	}

	public boolean stop() {
		pickupControl.set(0.0);
		return true;
	}

	public boolean reverse() {
		pickupControl.set(-1.0);
		return true;
	}
}
