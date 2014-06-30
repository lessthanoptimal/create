package roboticinception;

import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * @author Peter Abeles
 */
public class TestMiscOps {

	/**
	 * Trivial case with no rotation
	 */
	@Test
	public void cameraToPlane_trivial() {
		PlaneGeneral3D_F64 plane = new PlaneGeneral3D_F64(0,1,0,2);
		Se3_F64 found = MiscOps.cameraToPlane(plane);

		// check translation
		assertEquals(0,found.getT().x,1e-8);
		assertEquals(0,found.getT().y,1e-8);
		assertEquals(2,found.getT().z,1e-8);

		// check rotation
		Point3D_F64 p = SePointOps_F64.transform(found,new Point3D_F64(0,0,1),null);

		assertEquals(1,p.x,1e-8);
		assertEquals(0,p.y,1e-8);
		assertEquals(2,p.z,1e-8);
	}

	/**
	 * Camera is tilted down towards the ground
	 */
	@Test
	public void cameraToPlane_tilted() {
		Vector3D_F64 n = new Vector3D_F64(0,1,1);
		n.normalize();

		PlaneGeneral3D_F64 plane = new PlaneGeneral3D_F64(n.x,n.y,n.z,2);
		Se3_F64 found = MiscOps.cameraToPlane(plane);

		// check translation
		assertEquals(0,found.getT().x,1e-8);
		assertEquals(0,found.getT().y,1e-8);
		assertEquals(2,found.getT().z,1e-8);

		// check rotation
		Point3D_F64 p = SePointOps_F64.transform(found,new Point3D_F64(0,0,3),null);

		double theta = Math.PI/4;
		assertEquals(3*Math.cos(theta),p.x,1e-8);
		assertEquals(0,p.y,1e-8);
		assertEquals(2-3*Math.sin(theta),p.z,1e-8);
	}
}
