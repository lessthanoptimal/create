package roboticinception.autocalib;

import georegression.metric.UtilAngle;
import georegression.struct.point.Vector2D_F64;
import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.DistanceFromModel;

import java.util.List;

/**
 * @author Peter Abeles
 */
public class RotationChunkDistance implements DistanceFromModel<Se2_F64,Chunk> {

	double expected;

	@Override
	public void setModel(Se2_F64 motion) {
		expected = motion.getYaw();
	}

	@Override
	public double computeDistance(Chunk chunk) {
		Vector2D_F64 global = chunk.sensorB_Nto0.getTranslation();
		Vector2D_F64 child = chunk.sensorA_Nto0.getTranslation();

		double angleG = Math.atan2(global.getY(), global.getX());
		double angleC = Math.atan2(child.getY(), child.getX());

		double rotation = UtilAngle.distanceCCW(angleC, angleG);

		return UtilAngle.dist(expected,rotation);
	}

	@Override
	public void computeDistance(List<Chunk> chunks, double[] distance) {
		for (int i = 0; i < chunks.size(); i++) {
			distance[i] = computeDistance(chunks.get(i));
		}
	}
}
