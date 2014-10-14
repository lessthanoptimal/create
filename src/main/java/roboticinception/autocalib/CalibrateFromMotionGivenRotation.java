package roboticinception.autocalib;

import georegression.fitting.se.ModelManagerSe2_F64;
import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.lmeds.LeastMedianOfSquares;

/**
 * Estimates the translation between two reference frames given the rotation between the two.  The observed motion
 * must contain a rotational component or else a singularity will be encountered.
 *
 * @author Peter Abeles
 */
public class CalibrateFromMotionGivenRotation extends CalibrateBase {

	// minimum rotation in radians
	double minimumRotation;

	public CalibrateFromMotionGivenRotation( int robustIterations , double knownRotationBtoA , double minimumRotation) {
		robustEstimator = new LeastMedianOfSquares<Se2_F64,Chunk>(
				234234,robustIterations,new ModelManagerSe2_F64(),
				new TranslationFromMotionGivenRotation(knownRotationBtoA),new TranslationChunkDistance());

		this.minimumRotation = minimumRotation;
	}

	@Override
	protected boolean isChunkInvalid(Chunk chunk) {
		return false;
	}

	@Override
	protected boolean isChunkGood(Chunk chunk) {
		// TODO rotation should be about the same too ...  Cause to be invalid?
		return isEnoughMotion(chunk.sensorB_Nto0) && isEnoughMotion(chunk.sensorA_Nto0);
	}

	protected boolean isEnoughMotion( Se2_F64 motion ) {
		return Math.abs(motion.getYaw()) > minimumRotation;
	}

}
