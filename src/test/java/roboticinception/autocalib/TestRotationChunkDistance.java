package roboticinception.autocalib;

import georegression.struct.se.Se2_F64;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * @author Peter Abeles
 */
public class TestRotationChunkDistance {
	@Test
	public void basic() {

		Chunk chunk = new Chunk();
		chunk.global.set(1,0,0.1);
		chunk.child.set(1,1,0.1);

		RotationChunkDistance alg = new RotationChunkDistance();

		alg.setModel(new Se2_F64(0,0,-Math.PI/4.0));

		assertEquals(0,alg.computeDistance(chunk),1e-8);

		chunk.global.set(1,1,0.1);
		chunk.child.set(1,0,0.1);
		assertEquals(Math.PI/2.0,alg.computeDistance(chunk),1e-8);
	}
}