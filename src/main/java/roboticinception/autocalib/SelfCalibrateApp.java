package roboticinception.autocalib;

/**
 * @author Peter Abeles
 */
public class SelfCalibrateApp {
	public static void main(String[] args) {
		String directory = "/home/pja/projects/create/log/10-01-2014-15:33:23/";


		String logRPLIDAR = directory+"log_rplidar.txt";
		String logOdometry = directory+"log_odometry.txt";

		CreateOdometryMotion odom = new CreateOdometryMotion();
		RpLidarMotionIcp rplidar = new RpLidarMotionIcp();

		SelfCalibrateSensorKinematics alg = new SelfCalibrateSensorKinematics();

		alg.setParent(odom, logOdometry, null);
		alg.addChild(rplidar, logRPLIDAR, null);

		alg.estimateFromLinear();
	}
}
