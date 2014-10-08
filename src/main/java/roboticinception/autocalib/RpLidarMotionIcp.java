package roboticinception.autocalib;

import bubo.clouds.fit.Lrf2dScanToScan;
import bubo.clouds.fit.s2s.Lrf2dScanToScan_LocalICP;
import bubo.desc.sensors.lrf2d.Lrf2dMeasurement;
import bubo.log.LogMotion2;
import bubo.log.MotionInterpolateForward2;
import bubo.struct.StoppingCondition;
import georegression.struct.se.Se2_F64;
import roboticinception.log.RpLidarParser;
import roboticinception.lrf.UtilRpLidar;
import roboticinception.rplidar.RpLidarScan;

import java.io.IOException;

/**
 * @author Peter Abeles
 */
public class RpLidarMotionIcp implements EstimateMotion2D<String> {

	String logName;

	Lrf2dScanToScan scanMatching = new Lrf2dScanToScan_LocalICP(new StoppingCondition(50, 0.00001), 300, 0.3);

	RpLidarScan scan = new RpLidarScan();

	@Override
	public void setDataLog(String logName) {
		this.logName = logName;
	}

	@Override
	public LogMotion2 computeNoHint() {

		LogMotion2 output = new LogMotion2();

		RpLidarParser parser = new RpLidarParser(logName);

		Lrf2dMeasurement meas = null;

		scanMatching.setSensorParam(UtilRpLidar.createParam());

		Se2_F64 currentToWorld = new Se2_F64();
		Se2_F64 tmp = new Se2_F64();

		boolean first= true;
		while( parser.hasNext() ) {
			try {
				parser.next(scan);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}

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

					long time = -1;
					for (int i = 0; i < RpLidarScan.N; i++) {
						if( scan.distance[i] > 0 ) {
							time = scan.time[i];
							break;
						}
					}

					output.add(time / 1000.0, currentToWorld);
				} else {
					System.out.println("matching failed");
				}
				scanMatching.assignSourceToDestination();
			}
		}

		return output;
	}

	@Override
	public LogMotion2 computeWithHint(MotionInterpolateForward2 hint) {
		return null;
	}
}
