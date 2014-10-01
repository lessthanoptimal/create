package roboticinception.autocalib;

import bubo.log.LogMotion2;
import bubo.log.MotionInterpolateForward2;
import georegression.fitting.se.ModelManagerSe2_F64;
import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.lmeds.LeastMedianOfSquares;
import org.ddogleg.struct.FastQueue;

/**
 * Given two sets of odometry data from when the robot is moving in a straight line (translational only) compute the
 * rotation angle to align the 'child' to the 'global' reference frame.  Only a rotation can be found using this
 * technique.  To find the offset the robot needs to rotate.
 *
 * @author Peter Abeles
 */
public class CalibrateFromTranslation {

	// The estimated distanced traveled must be more than this for it to be used
	double minimumDistance;
	// maximum rotation in radians
	double maxRotation;

	double start;
	double stop;

	FastQueue<Chunk> chunks = new FastQueue<>(Chunk.class,true);

	LeastMedianOfSquares<Se2_F64,Chunk> robustEstimator;
	Se2_F64 found;

	public CalibrateFromTranslation( int robustIterations , double minimumDistance , double maxRotation) {
		robustEstimator = new LeastMedianOfSquares<Se2_F64,Chunk>(
				234234,robustIterations,new ModelManagerSe2_F64(),
				new RotationFromTranslation(),new RotationChunkDistance());

		this.minimumDistance = minimumDistance;
		this.maxRotation = maxRotation;
	}

	public boolean process( LogMotion2 global , LogMotion2 child ) {
		// find time period when the two logs over lap
		computeOverlapAndPeriod(global,child);

		// break the data up into a sequence of observations.  Each one can be used to estimate the motion
		// and be validated against all the others
		createChunks(global,child);

		// solve for N chunk - select using LSMED
		if( !robustEstimator.process(chunks.toList()) )
			return false;

		found = robustEstimator.getModelParameters();

		return true;
	}

	private void computeOverlapAndPeriod(LogMotion2 global , LogMotion2 child) {
		double startG = global.getHistory().get(0).time;
		double endG = global.getHistory().get(global.getHistory().size()-1).time;

		double startC = child.getHistory().get(0).time;
		double endC = child.getHistory().get(child.getHistory().size()-1).time;

		start = Math.max(startG,startC);
		stop = Math.min(endG,endC);
	}

	private void createChunks(LogMotion2 global , LogMotion2 child) {

		MotionInterpolateForward2 interpGlobal = new MotionInterpolateForward2();
		MotionInterpolateForward2 interpChild = new MotionInterpolateForward2();

		interpGlobal.setLog(global);
		interpChild.setLog(child);

		Se2_F64 beforeG = new Se2_F64();
		Se2_F64 beforeC = new Se2_F64();

		interpGlobal.lookup(start, beforeG);
		interpChild.lookup(start, beforeC);

		Se2_F64 currentG = new Se2_F64();
		Se2_F64 currentC = new Se2_F64();

		Se2_F64 inv = new Se2_F64();

		chunks.reset();

		boolean first = true;
		for (int i = 0; i < child.getHistory().size(); i++) {
			LogMotion2.Element e = child.getHistory().get(i);

			if( e.time < start )
				continue;
			if( e.time > stop )
				break;

			if( first ) {
				first = false;
				interpGlobal.lookup(start, beforeG);
				beforeC.set(e.motion);
				continue;
			} else {
				interpGlobal.lookup(e.time, currentG);
				currentC.set(e.motion);
			}

			Chunk chunk = chunks.grow();
			beforeG.invert(inv);
			inv.concat(currentG, chunk.global);

			beforeC.invert(inv);
			inv.concat(currentC, chunk.child);

			// if there is a big rotation completely discard the data
			if( isRotationLarge(chunk.global) || isRotationLarge(chunk.child)) {
				beforeG.set(currentG);
				beforeC.set(currentC);
				chunks.removeTail();
			} else if( isEnoughMotion(chunk.global) && isEnoughMotion(chunk.child)) {
				// todo the length should be about the same too....

				// it has translated enough to accept the data
				beforeG.set(currentG);
				beforeC.set(currentC);
			} else {
				// just skip over the data and see if its better when more is added
				chunks.removeTail();
			}
		}

		System.out.println("Total Chunks = "+chunks.size);

	}

	protected boolean isRotationLarge( Se2_F64 motion ) {
		return Math.abs(motion.getYaw()) > maxRotation;
	}

	protected boolean isEnoughMotion( Se2_F64 motion ) {
		return motion.getTranslation().norm() >= minimumDistance;
	}


	public Se2_F64 getFound() {
		return found;
	}
}

