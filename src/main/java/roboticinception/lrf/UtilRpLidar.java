package roboticinception.lrf;

import bubo.desc.sensors.lrf2d.Lrf2dMeasurement;
import bubo.desc.sensors.lrf2d.Lrf2dParam;
import roboticinception.rplidar.RpLidarScan;

/**
 * @author Peter Abeles
 */
public class UtilRpLidar {

	public static Lrf2dParam createParam() {

		double sweep = -2.0*Math.PI*(RpLidarScan.N-1)/RpLidarScan.N;

		return new Lrf2dParam("RP-LIDAR",0,sweep,RpLidarScan.N,5,0,0);
	}


	/**
	 * Converts into a generic Lrf2dMeasurement where range is in meters
	 */
	public static Lrf2dMeasurement convertM( RpLidarScan input , Lrf2dMeasurement output ) {
		if( output == null )
			output = new Lrf2dMeasurement(input.quality.length);

		for (int i = 0; i < input.quality.length; i++) {
			output.meas[i] = input.distance[i]/ 4000.0;
		}

		return output;
	}


}
