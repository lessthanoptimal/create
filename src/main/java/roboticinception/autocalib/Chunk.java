package roboticinception.autocalib;

import georegression.struct.se.Se2_F64;

/**
 * Contains the observed relative motion in the gobal and child reference frame
 *
 * @author Peter Abeles
 */
public class Chunk {
	Se2_F64 global = new Se2_F64();
	Se2_F64 child = new Se2_F64();
}
