package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LogitechController extends Joystick {
	public LogitechController(int port) {
		super(port);
	}

	/**
	 * Raw axis for xboxController & Logitech Controller
	 * someone find a pic of the dual action 
	 * 
	 */
	public static final int leftStickX = 0, leftStickY = 1, rightStickX = 2, rightStickY = 3;


	private double deadZone(double raw) {
		if (Math.abs(raw) < 0.1) {
			return 0.0;
		}
		return raw;
	}

	public double getLeftX() {
		return deadZone(this.getRawAxis(leftStickX));

	}

	public double getLeftY() {
		return deadZone(this.getRawAxis(leftStickY));
	}

	public double getRightX() {
		return deadZone(this.getRawAxis(rightStickX));
	}

	public double getRightY() {
		return deadZone(this.getRawAxis(rightStickY));
	}

	public double getDirection() {
		return Math.toDegrees(Math.atan2(getRightX(), -getRightY()));
	}
	public boolean getTriggers(){
		return this.getRawButton(RobotMap.leftTrigger) || this.getRawButton(RobotMap.rightTrigger);
	}
	
	
}
