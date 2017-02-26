package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.Joystick;

public class XBoxController {
	/**
	 * Raw axis for xboxController
	 * https://cdn.instructables.com/FNC/IF2H/HV3Q6145/FNCIF2HHV3Q6145.MEDIUM.jpg?width=614
	 */
	public static final int leftStickX = 1, leftStickY = 2, rightStickX = 4, rightStickY = 5;

	private Joystick j;

	public XBoxController(int port) {
		this.j = new Joystick(port);
	}

	public XBoxController(Joystick j) {
		this.j = j;
	}

	public double getX() {
		return j.getRawAxis(leftStickX);
	}

	public double getY() {
		return j.getRawAxis(leftStickY);
	}

	public double getDirection() {
		return Math.toDegrees(Math.atan2(j.getRawAxis(rightStickX), j.getRawAxis(rightStickY)));
	}
}
