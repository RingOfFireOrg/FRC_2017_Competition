package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TurnToAngleController extends Joystick {
	public TurnToAngleController(int port) {
		super(port);
	}
	public int getPOV(){
		if(getRawButton(1)){
			return 0;
		}
		if(getRawButton(8)){
			return 45;
		}
		if(getRawButton(7)){
			return 90;
		}
		if(getRawButton(6)){
			return 135;
		}
		if(getRawButton(5)){
			return 180;
		}
		if(getRawButton(4)){
			return 225;
		}
		if(getRawButton(3)){
			return 270;
		}
		if(getRawButton(2)){
			return 315;
		}
		return -1;
	}
}
