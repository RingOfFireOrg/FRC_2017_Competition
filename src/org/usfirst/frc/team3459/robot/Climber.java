package org.usfirst.frc.team3459.robot;
import com.ctre.CANTalon;

public class Climber {
	CANTalon climbControl1;
	CANTalon climbControl2;
	public Climber() {
		climbControl1 = new CANTalon(RobotMap.climberMotor1);
		climbControl2 = new CANTalon(RobotMap.climberMotor2);	
	}
	
	public boolean start() {
		climbControl1.set(1.0); // TODO DONT KNOW WHAT SPEED IS NEEDED
		climbControl2.set(-1.0); // TODO DONT KNOW WHAT SPEED IS NEEDED. However, both motors must spin opposite directions (there is a better way to do this but IDK how)
		return true;
	}
	public boolean stop(){
		climbControl1.set(0.0);
		climbControl2.set(0.0);
		return true;
	}
	public boolean reverse(){
		climbControl1.set(-1.0);
		climbControl2.set(1.0);
		return true;
	}
}
