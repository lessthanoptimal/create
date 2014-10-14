package roboticinception.autocalib;

import georegression.fitting.se.ModelManagerSe2_F64;
import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.lmeds.LeastMedianOfSquares;

/**
 * Given two sets of odometry data from when the robot is moving in a straight line (translational only) compute the
 * rotation angle to align the 'child' to the 'global' reference frame.  Only a rotation can be found using this
 * technique.  To find the offset the robot needs to rotate.
 *
 * @author Peter Abeles
 */
public class CalibrateFromTranslation extends CalibrateBase {

	// The estimated distanced traveled must be more than this for it to be used
	double minimumDistance;
	// maximum rotation in radians
	double maxRotation;

	public CalibrateFromTranslation( int robustIterations , double minimumDistance , double maxRotation) {
		robustEstimator = new LeastMedianOfSquares<Se2_F64,Chunk>(
				234234,robustIterations,new ModelManagerSe2_F64(),
				new RotationFromTranslation(),new RotationChunkDistance());

		this.minimumDistance = minimumDistance;
		this.maxRotation = maxRotation;
	}

	@Override
	protected boolean isChunkInvalid(Chunk chunk) {
		return isRotationLarge(chunk.sensorB_Nto0) || isRotationLarge(chunk.sensorA_Nto0);
	}

	@Override
	protected boolean isChunkGood(Chunk chunk) {
		// todo the length should be about the same too ...
		return isEnoughMotion(chunk.sensorB_Nto0) && isEnoughMotion(chunk.sensorA_Nto0);
	}

	protected boolean isRotationLarge( Se2_F64 motion ) {
		return Math.abs(motion.getYaw()) > maxRotation;
	}

	protected boolean isEnoughMotion( Se2_F64 motion ) {
		return motion.getTranslation().norm() >= minimumDistance;
	}
}

