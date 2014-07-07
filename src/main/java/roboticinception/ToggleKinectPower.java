package roboticinception;

/**
 * @author Peter Abeles
 */
public class ToggleKinectPower {
	public static void main(String[] args) throws Exception {
		CreateDriver serial = new CreateDriver("/dev/ttyUSB0");
		serial.startTheRobot();
		serial.sendKinect(true);
		System.out.println("Done");
		System.exit(0);
	}
}
