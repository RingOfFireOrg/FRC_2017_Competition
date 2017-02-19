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
	public boolean getShooter()
	{
		return this.getRawButton(RobotMap.btnShooter);
	}
	public boolean getClimber()
	{
		return this.getRawButton(RobotMap.btnClimber);
	}
	public boolean getPickerUpper()
	{
		return this.getRawButton(RobotMap.btnPickupper);
	}
	public boolean getEmergencyReverse()
	{
		return this.getRawButton(RobotMap.btnEmergencyReverse);
	}

}
