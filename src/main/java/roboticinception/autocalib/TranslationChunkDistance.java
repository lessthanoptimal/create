package roboticinception.autocalib;

import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.DistanceFromModel;

import java.util.List;

/**
 * Computes the difference in translation vector when given the transform from child to global
 *
 * @author Peter Abeles
 */
public class TranslationChunkDistance implements DistanceFromModel<Se2_F64,Chunk> {

	Se2_F64 childToGlobal = new Se2_F64();
	Se2_F64 globalToChild = new Se2_F64();

	Se2_F64 global1ToChild0 = new Se2_F64();
	Se2_F64 global1To0 = new Se2_F64();

	@Override
	public void setModel(Se2_F64 childToGlobal) {
		this.childToGlobal.set(childToGlobal);
		childToGlobal.invert(globalToChild);
	}

	@Override
	public double computeDistance(Chunk chunk) {

		globalToChild.concat(chunk.sensorA_Nto0, global1ToChild0);
		global1ToChild0.concat(childToGlobal,global1To0);

		return global1To0.T.distance2(chunk.sensorB_Nto0.T);
	}

	@Override
	public void computeDistance(List<Chunk> chunks, double[] distance) {
		for (int i = 0; i < chunks.size(); i++) {
			distance[i] = computeDistance(chunks.get(i));
		}
	}
}
