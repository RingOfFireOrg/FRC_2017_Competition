package org.usfirst.frc.team3459.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Cameras {
	public enum CameraType {
		FRONT, BACK
	}

	UsbCamera fwdCamera;
	UsbCamera bkwdCamera;
	MjpegServer mjpegServer;

	public Cameras() {
        CameraServer cs = CameraServer.getInstance();  
		fwdCamera = new UsbCamera("USB Camera 0", 0);
		bkwdCamera = new UsbCamera("USB Camera 1", 1);
		cs.addCamera(fwdCamera);
		cs.addCamera(bkwdCamera);
		
		mjpegServer = cs.addServer("serve_USB Camera 0");	
		mjpegServer.setSource(fwdCamera);
	}

	public boolean changeCamera(CameraType camera) {
		switch (camera) {
		case FRONT:
			mjpegServer.setSource(fwdCamera);
			SmartDashboard.putString("camera", "front");
			break;

		case BACK:
			mjpegServer.setSource(bkwdCamera);
			SmartDashboard.putString("camera", "back");
			break;
		default:
			return false;

		}
		return true;
	}

}
