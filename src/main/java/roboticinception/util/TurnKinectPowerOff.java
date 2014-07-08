package roboticinception.util;

import roboticinception.CreateDriver;

/**
 * @author Peter Abeles
 */
public class TurnKinectPowerOff {
	public static void main(String[] args) throws Exception {
		CreateDriver serial = new CreateDriver("/dev/ttyUSB0");
		serial.startTheRobot();
		serial.sendKinect(false);
		Thread.sleep(500);
		System.out.println("Done");
		System.exit(0);
	}
}
