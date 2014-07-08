package roboticinception.util;

import georegression.struct.se.Se2_F64;
import roboticinception.CreateDriver;

/**
 * @author Peter Abeles
 */
public class CreateRobotHardwareChecks {
	public static void main( String[] args ) {
		try {
			CreateDriver serial = new CreateDriver("/dev/ttyUSB0");
			serial.startTheRobot();
			serial.sendWheelVelocity(200,100);
			long start = System.currentTimeMillis();
			Se2_F64 location = new Se2_F64();
			long updateTime = 0;
			while( start + 10000 > System.currentTimeMillis() ) {
				serial.getLocation(location);

				if( serial.getSensorUpdatedTime() != updateTime ) {
					updateTime = serial.getSensorUpdatedTime();
					System.out.println("------------------------------- "+updateTime);
					System.out.println("bump  left = " + serial.isBumpLeft() + " right " + serial.isBumpRight());
					System.out.println("drop  left = " + serial.isWheelDropLeft() + " right " + serial.isWheelDropRight());
					System.out.println("caster = " + serial.isWheelDropCaster());
					System.out.println("x = " + location.getX() + " y = " + location.getY() + " yaw = " + location.getYaw());
					System.out.println("Gryo " + serial.getCargobayAnalog() + "  battery = " + serial.getBatteryCharge());

					Thread.sleep(200);
				}
			}
			serial.shutdown();
			System.out.println("Done");
			System.exit(0);
		} catch( Exception e ) {
			e.printStackTrace();
		}
	}
}
