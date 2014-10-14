package roboticinception.autocalib;

import georegression.struct.se.Se2_F64;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * @author Peter Abeles
 */
public class TestTranslationChunkDistance {
	@Test
	public void computeDistance() {

		// manually construct the kinematics chain
		Se2_F64 sensorToBody = new Se2_F64(1,0,0);
		Se2_F64 body1To0 = new Se2_F64(0,2,0);
		Se2_F64 sensor1To0 = new Se2_F64();

		sensorToBody.concat(body1To0,null).concat(sensorToBody.invert(null),sensor1To0);

		Chunk chunk = new Chunk();
		chunk.sensorA_Nto0.set(sensor1To0);
		chunk.sensorB_Nto0.set(body1To0);


		TranslationChunkDistance alg = new TranslationChunkDistance();

		alg.setModel(sensorToBody);

		// perfect
		assertEquals(0, alg.computeDistance(chunk), 1e-8);

		// not perfect
		chunk.sensorB_Nto0.set(0, 1.5, 0);
		assertEquals(0.5 * 0.5, alg.computeDistance(chunk), 1e-8);
	}
}