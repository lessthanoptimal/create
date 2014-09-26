package roboticinception;

import georegression.geometry.GeometryMath_F64;
import georegression.metric.ClosestPoint3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import org.ejml.data.DenseMatrix64F;

/**
 * @author Peter Abeles
 */
public class MiscOps {

	public static void sleep( int seconds , String message ) {
		System.out.println(message);
		for (int i = 0; i < seconds; i++) {
			System.out.print("*");
			try {Thread.sleep(1000);} catch (InterruptedException ignore) {}
		}
		System.out.println();
	}

	public static Se3_F64 cameraToPlane( PlaneGeneral3D_F64 plane ) {
		Point3D_F64 closest = ClosestPoint3D_F64.closestPoint(plane, new Point3D_F64(), null);

		// pointing out of the camera
		Point3D_F64 closestZ = ClosestPoint3D_F64.closestPoint(plane,new Point3D_F64(0,0,1),null);
		// pointing to the right of the camera
		Point3D_F64 closestX = ClosestPoint3D_F64.closestPoint(plane,new Point3D_F64(1,0,0),null);

		Vector3D_F64 axisX = new Vector3D_F64(closestX.x-closest.x,closestX.y-closest.y,closestX.z-closest.z);
		Vector3D_F64 axisZ = new Vector3D_F64(closestZ.x-closest.x,closestZ.y-closest.y,closestZ.z-closest.z);
		Vector3D_F64 axisY = new Vector3D_F64(closest.x,closest.y,closest.z);

		axisX.normalize();
		axisZ.normalize();
		axisY.normalize();

		Se3_F64 ret = new Se3_F64();

		DenseMatrix64F R = ret.getR();
		axisX.scale(-1);
		axisY.scale(-1);
		R.set(0,0,axisZ.getX());R.set(0,1,axisZ.getY());R.set(0,2,axisZ.getZ());
		R.set(1,0,axisX.getX());R.set(1,1,axisX.getY());R.set(1,2,axisX.getZ());
		R.set(2,0,axisY.getX());R.set(2,1,axisY.getY());R.set(2,2,axisY.getZ());

		// closest is translation from plane to camera, change sign to go from camera to plane
		closest.scale(-1);
		GeometryMath_F64.mult(R, closest, ret.getT());

		return ret;
	}
}
