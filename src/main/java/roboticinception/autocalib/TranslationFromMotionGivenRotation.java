package roboticinception.autocalib;

import georegression.struct.se.Se2_F64;
import org.ddogleg.fitting.modelset.ModelGenerator;
import org.ejml.alg.fixed.FixedOps2;
import org.ejml.data.FixedMatrix2x2_64F;

import java.util.List;

/**
 * Given the observed motion being pure translation, estimate the rotational component of the rigid body transform
 * from child to global.
 *
 * @author Peter Abeles
 */
public class TranslationFromMotionGivenRotation implements ModelGenerator<Se2_F64,Chunk> {

	// known cosine and sine for rotation form a to b
	double outC,outS;

	FixedMatrix2x2_64F r_i_inv = new FixedMatrix2x2_64F();
	Se2_F64 bToA = new Se2_F64();

	public TranslationFromMotionGivenRotation( double rotationFromAToB ) {

		double rotationFromBToA = -rotationFromAToB;

		outC = Math.cos(rotationFromBToA);
		outS = Math.sin(rotationFromBToA);
	}

	@Override
	public boolean generate(List<Chunk> dataSet, Se2_F64 aToB) {

		Chunk chunk = dataSet.get(0);

		Se2_F64 a = chunk.child;
		Se2_F64 b = chunk.global;

		// R_(aj,ai) - I
		r_i_inv.a11 =  a.c - 1;
		r_i_inv.a12 = -a.s;
		r_i_inv.a21 =  a.s;
		r_i_inv.a22 =  a.c - 1;
		if( !FixedOps2.invert(r_i_inv,r_i_inv) )
			return false;

		// R_ab*tau_(bi,bj)
		double x1 = outC*b.T.x - outS*b.T.y;
		double y1 = outS*b.T.x + outC*b.T.y;

		// [x1,y1] - tau_(ai,aj)
		double x2 = x1 - a.T.x;
		double y2 = y1 - a.T.y;

		bToA.T.x = r_i_inv.a11*x2 + r_i_inv.a12*y2;
		bToA.T.y = r_i_inv.a21*x2 + r_i_inv.a22*y2;
		bToA.c = outC;
		bToA.s = outS;

		bToA.invert(aToB);

		return true;
	}

	@Override
	public int getMinimumPoints() {
		return 1;
	}
}
