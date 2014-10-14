package roboticinception.autocalib;

import georegression.struct.se.Se2_F64;

/**
 * Contains the observed relative motion between two sensors which are rigidly attached.  The observed motion
 * is from the sensors perspective and is from the current frame (N) to the sensor's origin.
 *
 * @author Peter Abeles
 */
public class Chunk {
	Se2_F64 sensorA_Nto0 = new Se2_F64();
	Se2_F64 sensorB_Nto0 = new Se2_F64();
}
