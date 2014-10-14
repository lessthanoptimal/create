package roboticinception.autocalib;

import georegression.metric.UtilAngle;
import georegression.struct.point.Vector2D_F64;
import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.ModelGenerator;

import java.util.List;

/**
 * Given the observed motion being pure translation, estimate the rotational component of the rigid body transform
 * from sensor B to sensor A.
 *
 * @author Peter Abeles
 */
public class RotationFromTranslation implements ModelGenerator<Se2_F64,Chunk> {
	@Override
	public boolean generate(List<Chunk> dataSet, Se2_F64 fromBtoA) {

		Chunk chunk = dataSet.get(0);

		Vector2D_F64 g = chunk.sensorB_Nto0.getTranslation();
		Vector2D_F64 c = chunk.sensorA_Nto0.getTranslation();

		double angleG = Math.atan2(g.getY(),g.getX());
		double angleC = Math.atan2(c.getY(),c.getX());

		double rotation = UtilAngle.distanceCCW(angleC,angleG);

		fromBtoA.set(0, 0, rotation);

		return true;
	}

	@Override
	public int getMinimumPoints() {
		return 1;
	}
}
