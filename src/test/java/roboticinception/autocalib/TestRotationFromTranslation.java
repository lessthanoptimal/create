package roboticinception.autocalib;

import georegression.struct.se.Se2_F64;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;

/**
 * @author Peter Abeles
 */
public class TestRotationFromTranslation {
	@Test
	public void basicCCW() {

		Chunk chunk = new Chunk();
		chunk.global.set(1,0,0.1);
		chunk.child.set(1,1,0.1);

		RotationFromTranslation alg = new RotationFromTranslation();

		List<Chunk> chunks = new ArrayList<Chunk>();
		chunks.add(chunk);

		Se2_F64 found = new Se2_F64();
		alg.generate(chunks,found);

		assertEquals(-Math.PI/4.0,found.getYaw(),1e-8);
	}

	@Test
	public void basicCW() {

		Chunk chunk = new Chunk();
		chunk.global.set(1,1,0.1);
		chunk.child.set(1,0,0.1);

		RotationFromTranslation alg = new RotationFromTranslation();

		List<Chunk> chunks = new ArrayList<Chunk>();
		chunks.add(chunk);

		Se2_F64 found = new Se2_F64();
		alg.generate(chunks,found);

		assertEquals(Math.PI/4.0,found.getYaw(),1e-8);
	}

	@Test
	public void getMinimumPoints() {
		RotationFromTranslation alg = new RotationFromTranslation();
		assertEquals(1,alg.getMinimumPoints());
	}
}