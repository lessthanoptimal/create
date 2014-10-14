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

		Se2_F64 sensorA0ToWorld = new Se2_F64(0.2,0.5,0.2);
		Se2_F64 sensorA1ToWorld = new Se2_F64(0.4,0.1,0.6);

		Se2_F64 sensorA_1to0 = sensorA1ToWorld.concat(sensorA0ToWorld.invert(null),null);

		Se2_F64 BtoA = new Se2_F64(0.2,-0.15,0.1);
		Se2_F64 AtoB = BtoA.invert(null);

		Se2_F64 sensorB0ToWorld = BtoA.concat(sensorA0ToWorld,null);
		Se2_F64 sensorB1ToWorld = BtoA.concat(sensorA1ToWorld,null);

		Se2_F64 sensorB_1to0 = sensorB1ToWorld.concat(sensorB0ToWorld.invert(null),null);

		Chunk chunk = new Chunk();
		chunk.sensorA_Nto0 = sensorA_1to0;
		chunk.sensorB_Nto0 = sensorB_1to0;

		TranslationFromMotionGivenRotation alg = new TranslationFromMotionGivenRotation(BtoA.getYaw());

		List<Chunk> chunks = new ArrayList<Chunk>();
		chunks.add(chunk);

		Se2_F64 foundBtoA = new Se2_F64();
		alg.generate(chunks, foundBtoA);

		assertEquals(BtoA.getX()  , foundBtoA.getX(), 1e-8);
		assertEquals(BtoA.getY()  , foundBtoA.getY(), 1e-8);
		assertEquals(BtoA.getYaw(), foundBtoA.getYaw(), 1e-8);
	}

	@Test
	public void getMinimumPoints() {
		TranslationFromMotionGivenRotation alg = new TranslationFromMotionGivenRotation(0);
		assertEquals(1,alg.getMinimumPoints());
	}
}