package org.usfirst.frc.team3459.robot;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

/**
 * Don't change the name of this or it won't work. (The manifest looks for "Robot")
 */
public class Robot extends IterativeRobot {
	
	/*
	 * Member variables go here 
	 */
	UltrasonicSensor rangeFinder = new UltrasonicSensor();
	HDrive driveTrain = new HDrive(RobotMap.leftMotor, RobotMap.rightMotor, RobotMap.shimmyMotor); // class that handles basic drive
	// operations

	Joystick leftStick = new Joystick(RobotMap.leftStick);
	Joystick rightStick = new Joystick(RobotMap.rightStick);
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {        

    }
      
    /**
     * This function is called once when we go into the teleop mode
     */
    public void teleopInit(){
    }
    
    /**
     * This function is called periodically during operator control (approx 20ms)
     */
    public void teleopPeriodic() {
		driveTrain.tankDrive(leftStick, rightStick);
		if(leftStick.getTrigger()) {
			driveTrain.shimmyLeft();
		}
		else if(rightStick.getTrigger()){
			driveTrain.shimmyRight();
		}
		else{
			driveTrain.shimmyStop();
		}
		SmartDashboard.putNumber("distance", rangeFinder.getDistance());
		
		
    }

    /**
     * This function is called once when we go into the Autonomous mode
     */
    public void autonomousInit(){
    }

    /**
     * This function is called periodically during autonomous control (approx 20ms)
     */
    public void autonomousPeriodic() {
     }

    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run(); // This makes sure the values of items are correct on the driver station during test mode.   
    }    
}

