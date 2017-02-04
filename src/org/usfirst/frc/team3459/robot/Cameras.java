package org.usfirst.frc.team3459.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.MjpegServer;

public class Cameras {
	public enum CameraType {
		FRONT, BACK
	}

	UsbCamera fwdCamera;
	UsbCamera bkwdCamera;
	MjpegServer mjpegServer;

	public Cameras() {
		fwdCamera = new UsbCamera("USB Camera 0", 0);
		bkwdCamera = new UsbCamera("USB Camera 1", 1);
		mjpegServer = new MjpegServer("serve_USB Camera 0", 1181);
		mjpegServer.setSource(fwdCamera);
	}

	public boolean changeCamera(CameraType camera) {
		switch (camera) {
		case FRONT:
			mjpegServer.setSource(fwdCamera);
			break;

		case BACK:
			mjpegServer.setSource(bkwdCamera);
			break;
		default:
			return false;

		}
		return true;
	}

}
