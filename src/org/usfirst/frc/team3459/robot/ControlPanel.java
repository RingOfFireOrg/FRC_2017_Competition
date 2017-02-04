package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ControlPanel extends Joystick {
	public ControlPanel(int port) {
		super(port);
	}

	public int getProgram()
	{
		int result = 0;
		
		if(this.getRawButton(RobotMap.selector_1)){
			result = result + 1;
		}
		if(this.getRawButton(RobotMap.selector_2)){
			result = result + 2;
		}
		if(this.getRawButton(RobotMap.selector_3)){
			result = result + 4;
		}
		return result;
	}
	public boolean getUpperLeft()
	{
		return this.getRawButton(RobotMap.btnUpperLeft);
	}
	public boolean getUpperRight()
	{
		return this.getRawButton(RobotMap.btnUpperRight);
	}
	public boolean getLowerLeft()
	{
		return this.getRawButton(RobotMap.btnLowerLeft);
	}
	public boolean getLowerRight()
	{
		return this.getRawButton(RobotMap.btnLowerRight);
	}

}
