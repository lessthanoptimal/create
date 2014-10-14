package roboticinception.autocalib;

import bubo.log.LogMotion2;
import bubo.log.MotionInterpolateForward2;
import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.lmeds.LeastMedianOfSquares;
import org.ddogleg.struct.FastQueue;

/**
 * @author Peter Abeles
 */
public abstract class CalibrateBase {

	double start;
	double stop;

	protected FastQueue<Chunk> chunks = new FastQueue<>(Chunk.class,true);

	protected LeastMedianOfSquares<Se2_F64,Chunk> robustEstimator;
	protected Se2_F64 foundBtoA;

	/**
	 *
	 * Inputs are logs describing the apparent motion from two sensors point of view.  it is assumed that there is
	 * a fixed rigid body transform from sensor A to B.  The transforms in the passed in logs go from current frame
	 * to the sensors origin.
	 *
	 * @param sensorA Log of apparent motion from sensor A's point of view.  Transforms from current to sensor origin.
	 * @param sensorB Log of apparent motion from sensor B's point of view.  Transforms from current to sensor origin.
	 * @return true if rigid transform from B to A could be found.
	 */
	public boolean process( LogMotion2 sensorA , LogMotion2 sensorB ) {
		// find time period when the two logs over lap
		computeOverlapAndPeriod(sensorA,sensorB);

		// break the data up into a sequence of observations.  Each one can be used to estimate the motion
		// and be validated against all the others
		createChunks(sensorA,sensorB);

		// solve for N chunk - select using LSMED
		if( !robustEstimator.process(chunks.toList()) )
			return false;

		foundBtoA = robustEstimator.getModelParameters();

		return true;
	}

	protected void computeOverlapAndPeriod(LogMotion2 sensorA , LogMotion2 sensorB) {
		double startG = sensorA.getHistory().get(0).time;
		double endG = sensorA.getHistory().get(sensorA.getHistory().size()-1).time;

		double startC = sensorB.getHistory().get(0).time;
		double endC = sensorB.getHistory().get(sensorB.getHistory().size()-1).time;

		start = Math.max(startG,startC);
		stop = Math.min(endG,endC);
	}

	protected void createChunks(LogMotion2 logA , LogMotion2 logB) {

		MotionInterpolateForward2 interpA = new MotionInterpolateForward2();
		MotionInterpolateForward2 interpB = new MotionInterpolateForward2();

		interpA.setLog(logA);
		interpB.setLog(logB);

		// reminder: Transforms in log are from current frame to sensor origin
		Se2_F64 poseA_0toOrig = new Se2_F64();
		Se2_F64 poseB_0toOrig = new Se2_F64();

		interpA.lookup(start, poseA_0toOrig);
		interpB.lookup(start, poseB_0toOrig);

		Se2_F64 poseA_1toOrig = new Se2_F64();
		Se2_F64 poseB_1toOrig = new Se2_F64();

		Se2_F64 OrigTo0 = new Se2_F64();

		chunks.reset();

		boolean first = true;
		for (int i = 0; i < logB.getHistory().size(); i++) {
			LogMotion2.Element e = logB.getHistory().get(i);

			if( e.time < start )
				continue;
			if( e.time > stop )
				break;

			if( first ) {
				first = false;
				interpA.lookup(start, poseA_0toOrig);
				poseB_0toOrig.set(e.motion);
				continue;
			} else {
				interpA.lookup(e.time, poseA_1toOrig);
				poseB_1toOrig.set(e.motion);
			}

			// change the origin to the start of the frame
			Chunk chunk = chunks.grow();
			poseA_0toOrig.invert(OrigTo0);
			poseA_1toOrig.concat(OrigTo0, chunk.sensorA_Nto0);

			poseB_0toOrig.invert(OrigTo0);
			poseB_1toOrig.concat(OrigTo0, chunk.sensorB_Nto0);

			// if there is a big rotation completely discard the data
			if( isChunkInvalid(chunk) ) {
				poseA_0toOrig.set(poseA_1toOrig);
				poseB_0toOrig.set(poseB_1toOrig);
				chunks.removeTail();
			} else if( isChunkGood(chunk)) {
				// it has translated enough to accept the data
				poseA_0toOrig.set(poseA_1toOrig);
				poseB_0toOrig.set(poseB_1toOrig);
			} else {
				// just skip over the data and see if its better when more is added
				chunks.removeTail();
			}
		}

//		System.out.println("Total Chunks = "+chunks.size);

	}

	protected abstract boolean isChunkInvalid( Chunk chunk );

	protected abstract boolean isChunkGood( Chunk chunk );

	public Se2_F64 getTransformSensorBtoA() {
		return foundBtoA;
	}
}
