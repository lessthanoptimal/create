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
		check(0.3);
	}

	@Test
	public void basicCW() {
		check(-0.4);
	}

	public void check( double heading ) {
		Se2_F64 child0ToWorld = new Se2_F64(0.2,0.5,heading);
		Se2_F64 child1ToWorld = new Se2_F64(0.4,0.1,heading);

		Se2_F64 globalToChild = new Se2_F64(0.1,-0.2,0);

		Se2_F64 childToGlobal = globalToChild.invert(null);

		Se2_F64 global0ToWorld = globalToChild.concat(child0ToWorld,null);
		Se2_F64 global1ToWorld = globalToChild.concat(child1ToWorld,null);

		Chunk chunk = new Chunk();
		child0ToWorld.concat(child1ToWorld.invert(null), chunk.sensorA_Nto0);
		global0ToWorld.concat(global1ToWorld.invert(null), chunk.sensorB_Nto0);

		RotationFromTranslation alg = new RotationFromTranslation();

		List<Chunk> chunks = new ArrayList<Chunk>();
		chunks.add(chunk);

		Se2_F64 found = new Se2_F64();
		alg.generate(chunks,found);

		assertEquals(0, found.getX(), 1e-8);
		assertEquals(0, found.getY(), 1e-8);
		assertEquals(childToGlobal.getYaw(), found.getYaw(), 1e-8);
	}

	@Test
	public void getMinimumPoints() {
		RotationFromTranslation alg = new RotationFromTranslation();
		assertEquals(1,alg.getMinimumPoints());
	}
}