package org.usfirst.frc.team3459.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.CANTalon; 

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {
	HDrive myRobot = new HDrive(RobotMap.leftMotor, RobotMap.rightMotor, RobotMap.shimmyMotor); // class that handles basic drive
												// operations
		
	Joystick leftStick = new Joystick(RobotMap.leftStick);
	Joystick rightStick = new Joystick(RobotMap.rightStick);
	
	public Robot() {
		myRobot.setExpiration(0.1);
	}

	/**
	 * Runs the motors with tank steering.
	 */
	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			myRobot.tankDrive(leftStick, rightStick);
			Timer.delay(0.005); // wait for a motor update time
			if(leftStick.getTrigger()) {
				myRobot.shimmyLeft();
			}
			else if(rightStick.getTrigger()){
				myRobot.shimmyRight();
			}
			else{
				myRobot.shimmyStop();
			}
		}

		}
	}

