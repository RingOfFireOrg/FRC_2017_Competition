package org.usfirst.frc.team3459.robot;
import com.ctre.CANTalon;

public class Climber {
	CANTalon climbControl;
	
	public Climber() {
		climbControl = new CANTalon(RobotMap.climberMotor);
	}

	public boolean startClimber() {
		climbControl.set(1.0); // TODO DONT KNOW WHAT SPEED IS NEEDED
		return true;
	}
	public boolean stopClimber(){
		climbControl.set(0.0);
		return true;
	}
}
