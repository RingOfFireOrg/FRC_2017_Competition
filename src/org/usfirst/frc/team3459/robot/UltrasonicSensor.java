package org.usfirst.frc.team3459.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor {
	AnalogInput sensor;
	
    public UltrasonicSensor(int port){
    	sensor = new AnalogInput(port);
    }
    
	public double getDistance() {
		double voltsPerInch = 0.0098;
		return sensor.getAverageVoltage() / voltsPerInch;
	}
}
