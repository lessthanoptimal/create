package roboticinception.autocalib;

import bubo.log.LogMotion2;
import bubo.log.MotionInterpolateForward2;

/**
 * Estimates motion from a data log
 *
 * @author Peter Abeles
 */
public interface EstimateMotion2D<DataLog> {


	public void setDataLog( DataLog data );

	public LogMotion2 computeNoHint();

	public LogMotion2 computeWithHint( MotionInterpolateForward2 hint );
}
