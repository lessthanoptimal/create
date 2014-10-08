package roboticinception.autocalib;

import georegression.struct.se.Se2_F64;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;

/**
 * @author Peter Abeles
 */
public class TestTranslationFromMotionGivenRotation {
	@Test
	public void full() {

		Se2_F64 child0ToWorld = new Se2_F64(0.2,0.5,0.2);
		Se2_F64 child1ToWorld = new Se2_F64(0.4,0.1,0.6);

		Se2_F64 globalToChild = new Se2_F64(0.1,-0.2,0.5);

		Se2_F64 childToGlobal = globalToChild.invert(null);

		Se2_F64 global0ToWorld = globalToChild.concat(child0ToWorld,null);
		Se2_F64 global1ToWorld = globalToChild.concat(child1ToWorld,null);

		Chunk chunk = new Chunk();
		child0ToWorld.concat(child1ToWorld.invert(null), chunk.child);
		global0ToWorld.concat(global1ToWorld.invert(null), chunk.global);

		TranslationFromMotionGivenRotation alg = new TranslationFromMotionGivenRotation(childToGlobal.getYaw());

		List<Chunk> chunks = new ArrayList<Chunk>();
		chunks.add(chunk);

		Se2_F64 found = new Se2_F64();
		alg.generate(chunks,found);

		assertEquals(childToGlobal.getX(), found.getX(), 1e-8);
		assertEquals(childToGlobal.getY(), found.getY(), 1e-8);
		assertEquals(childToGlobal.getYaw(), found.getYaw(), 1e-8);
	}

	@Test
	public void getMinimumPoints() {
		TranslationFromMotionGivenRotation alg = new TranslationFromMotionGivenRotation(0);
		assertEquals(1,alg.getMinimumPoints());
	}
}