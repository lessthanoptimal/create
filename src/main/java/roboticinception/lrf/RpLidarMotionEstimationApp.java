package roboticinception.lrf;

import bubo.clouds.fit.Lrf2dScanToScan;
import bubo.clouds.fit.s2s.Lrf2dScanToScan_LocalICP;
import bubo.desc.sensors.lrf2d.Lrf2dMeasurement;
import bubo.struct.StoppingCondition;
import georegression.struct.se.Se2_F64;
import roboticinception.rplidar.RpLidarScan;

import java.io.IOException;

/**
 * @author Peter Abeles
 */
public class RpLidarMotionEstimationApp {

	Lrf2dScanToScan scanMatching = new Lrf2dScanToScan_LocalICP(new StoppingCondition(20, 0.0001), 50, 0.30);

	RpLidarScan scan = new RpLidarScan();

	public void process( String logName ) throws IOException {

		RpLidarParser parser = new RpLidarParser(logName);

		Lrf2dMeasurement meas = null;

		scanMatching.setSensorParam(UtilRpLidar.createParam());

		Se2_F64 currentToWorld = new Se2_F64();
		Se2_F64 tmp = new Se2_F64();

		boolean first= true;
		while( parser.hasNext() ) {
			parser.next(scan);

			meas = UtilRpLidar.convertM(scan,meas);

			if( first ) {
				first = false;
				scanMatching.setDestination(meas.meas);
			} else {
				scanMatching.setSource(meas.meas);
				if( scanMatching.process(null) ) {
					Se2_F64 currToPrev = scanMatching.getSourceToDestination();
					currToPrev.concat(currentToWorld, tmp);
					currentToWorld.set(tmp);
					currentToWorld.print();
				} else {
					System.out.println("matching failed");
				}
				scanMatching.assignSourceToDestination();
			}
		}
		System.out.println("Done");
	}

	public static void main(String[] args) throws IOException {
		RpLidarMotionEstimationApp app = new RpLidarMotionEstimationApp();

		app.process("/home/pja/projects/create/log/09-26-2014-16:59:51/log_rplidar.txt");
	}
}
