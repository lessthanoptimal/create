package roboticinception.autocalib;

import georegression.metric.UtilAngle;
import georegression.struct.point.Vector2D_F64;
import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.ModelGenerator;

import java.util.List;

/**
 * Given the observed motion being pure translation, estimate the rotational component of the rigid body transform
 * from child to global.
 *
 * @author Peter Abeles
 */
public class RotationFromTranslation implements ModelGenerator<Se2_F64,Chunk> {
	@Override
	public boolean generate(List<Chunk> dataSet, Se2_F64 output) {

		Chunk chunk = dataSet.get(0);

		Vector2D_F64 g = chunk.global.getTranslation();
		Vector2D_F64 c = chunk.child.getTranslation();

		double angleG = Math.atan2(g.getY(),g.getX());
		double angleC = Math.atan2(c.getY(),c.getX());

		double rotation = UtilAngle.distanceCCW(angleC,angleG);

		output.set(0,0,rotation);

		return true;
	}

	@Override
	public int getMinimumPoints() {
		return 1;
	}
}
