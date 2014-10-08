package roboticinception.autocalib;

import bubo.log.LogMotion2;
import bubo.log.MotionInterpolateForward2;
import georegression.struct.se.Se2_F64;
import roboticinception.CreateOdometry;
import roboticinception.log.CreateOdometryParser;

import java.io.IOException;

/**
 * Parses a create odometry log file and returns the motion log without modifications
 *
 * @author Peter Abeles
 */
public class CreateOdometryMotion implements EstimateMotion2D<String> {

	LogMotion2 motion;

	@Override
	public void setDataLog(String fileName) {
		motion = new LogMotion2();

		CreateOdometryParser parser = new CreateOdometryParser(fileName);

		CreateOdometry o = new CreateOdometry();
		Se2_F64 pose = new Se2_F64();
		while( parser.hasNext() ) {
			try {
				parser.next(o);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}

			pose.set(o.x,o.y,o.theta);
			motion.add(o.time/1000.0,pose);
		}
	}

	@Override
	public LogMotion2 computeNoHint() {
		return motion;
	}

	@Override
	public LogMotion2 computeWithHint(MotionInterpolateForward2 hint) {
		return motion;
	}
}
