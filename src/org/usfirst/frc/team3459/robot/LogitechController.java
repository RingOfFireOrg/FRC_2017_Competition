package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechController extends Joystick {
	public LogitechController(int port) {
		super(port);
	}
	/**
	 * Raw axis for xboxController & Logitech Controller
	 * https://cdn.instructables.com/FNC/IF2H/HV3Q6145/FNCIF2HHV3Q6145.MEDIUM.jpg?width=614
	 */
	public static final int leftStickX = 0, leftStickY = 1, rightStickX = 4, rightStickY = 5;
	

	public double getLeftX() {
		return this.getRawAxis(leftStickX);
	}

	public double getLeftY() {
		return this.getRawAxis(leftStickY);
	}

	public double getRightX() {
		return this.getRawAxis(rightStickX);
	}
	
	public double getRightY() {
		return this.getRawAxis(rightStickY);
	}

	public double getDirection() {
		return Math.toDegrees(Math.atan2(this.getRawAxis(rightStickX), -this.getRawAxis(rightStickY)));
	}
		
	
	
}		
