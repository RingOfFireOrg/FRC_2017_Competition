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
	public static final int leftStickX = 12, leftStickY = 2, rightStickX = 0, rightStickY = 1;
	
	private Joystick j;

	public double getLeftX() {
		return j.getRawAxis(leftStickX);
	}

	public double getLeftY() {
		return j.getRawAxis(leftStickY);
	}

	public double getRightX() {
		return j.getRawAxis(rightStickX);
	}
	
	public double getRightY() {
		return j.getRawAxis(rightStickY);
	}

	public double getDirection() {
		return Math.toDegrees(Math.atan2(j.getRawAxis(rightStickX), j.getRawAxis(-rightStickY)));
	}
		
	
	
}		
