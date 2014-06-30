package roboticinception;

import boofcv.alg.depth.VisualDepthOps;
import boofcv.alg.distort.RemoveRadialPtoN_F64;
import boofcv.gui.image.ShowImages;
import boofcv.gui.image.VisualizeImageData;
import boofcv.io.UtilIO;
import boofcv.io.image.UtilImageIO;
import boofcv.openkinect.UtilOpenKinect;
import boofcv.struct.FastQueueArray_I32;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.calib.VisualDepthParameters;
import boofcv.struct.image.ImageUInt16;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;
import georegression.fitting.plane.FitPlane3D_F64;
import georegression.fitting.plane.ModelManagerPlaneGeneral3D_F64;
import georegression.geometry.UtilPlane3D_F64;
import georegression.metric.Distance3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.fitting.modelset.DistanceFromModel;
import org.ddogleg.fitting.modelset.ModelGenerator;
import org.ddogleg.fitting.modelset.ModelManager;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.fitting.modelset.lmeds.LeastMedianOfSquares;
import org.ddogleg.struct.FastQueue;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.List;

/**
 * @author Peter Abeles
 */
public class DetectPlanePointCloudApp {
	public static void main(String[] args) throws IOException {
		VisualDepthParameters intrinsic = UtilIO.loadXML("data/intrinsic_kinect.xml");

		String nameRgb = "log/rgb0000015.ppm";
		String nameDepth = "log/depth0000015.depth";

		ImageUInt16 depth = new ImageUInt16(1,1);
		MultiSpectral<ImageUInt8> rgb = new MultiSpectral<ImageUInt8>(ImageUInt8.class,1,1,3);

		UtilImageIO.loadPPM_U8(nameRgb, rgb, null);
		UtilOpenKinect.parseDepth(nameDepth, depth, null);

		FastQueue<Point3D_F64> cloud = new FastQueue<Point3D_F64>(Point3D_F64.class,true);
		FastQueueArray_I32 cloudColor = new FastQueueArray_I32(3);

		VisualDepthOps.depthTo3D(intrinsic.visualParam, rgb, depth, cloud, cloudColor);

		// convert points to meters
		for ( Point3D_F64 p : cloud.toList() ) {
			p.scale(1.0 / 1000.0);
		}

		ModelManager<PlaneGeneral3D_F64> modelManager = new ModelManagerPlaneGeneral3D_F64();
		PlaneGenerator generator = new PlaneGenerator();
		PlaneDistance errorMetric = new PlaneDistance();

		ModelMatcher<PlaneGeneral3D_F64,Point3D_F64> fitter =
				new LeastMedianOfSquares<PlaneGeneral3D_F64, Point3D_F64>(0xDEADBEEF,300,0.1,0,
						modelManager,generator,errorMetric);

		fitter.process(cloud.toList());

		PlaneGeneral3D_F64 foundPlane = fitter.getModelParameters();

		BufferedImage buffDepth = new BufferedImage(depth.width,depth.height,BufferedImage.TYPE_INT_RGB);
		VisualizeImageData.disparity(depth, buffDepth, 0, UtilOpenKinect.FREENECT_DEPTH_MM_MAX_VALUE, 0);

		RemoveRadialPtoN_F64 p2n = new RemoveRadialPtoN_F64();
		IntrinsicParameters ip = intrinsic.getVisualParam();
		p2n.set(ip.fx,ip.fy,ip.skew,ip.cx,ip.cy,ip.radial);

		Point2D_F64 n = new Point2D_F64();
		Point3D_F64 p = new Point3D_F64();
		for( int y = 0; y < depth.height; y++ ) {
			int index = depth.startIndex + y*depth.stride;
			for( int x = 0; x < depth.width; x++ ) {
				int mm = depth.data[index++] & 0xFFFF;


				// skip pixels with no depth information
				if( mm == 0 ) {
					continue;
				}

				double m = mm/1000.0;

				// this could all be precomputed to speed it up
				p2n.compute(x,y,n);

				p.z = m;
				p.x = n.x*m;
				p.y = n.y*m;

				double distance = Distance3D_F64.distance(foundPlane,p);

				if( distance < 0.03 ) {
					buffDepth.setRGB(x, y, 0x00FF00);
				}
			}
		}

		ShowImages.showWindow(buffDepth,"Disparity plane fit");

		Se3_F64 found = MiscOps.cameraToPlane(foundPlane);
		found.print();

		UtilIO.saveXML(found,"cameraToGround.xml");

	}

	public static class PlaneDistance implements DistanceFromModel<PlaneGeneral3D_F64,Point3D_F64> {

		PlaneGeneral3D_F64 plane;

		@Override
		public void setModel(PlaneGeneral3D_F64 plane) {
			this.plane = plane;
		}

		@Override
		public double computeDistance(Point3D_F64 pt) {
			return Math.abs(Distance3D_F64.distance(plane, pt));
		}

		@Override
		public void computeDistance(List<Point3D_F64> points, double[] distance) {
			for (int i = 0; i < points.size(); i++) {
				distance[i] = computeDistance(points.get(i));
			}
		}
	}

	public static class PlaneGenerator implements ModelGenerator<PlaneGeneral3D_F64,Point3D_F64>
	{
		FitPlane3D_F64 fit = new FitPlane3D_F64();
		PlaneNormal3D_F64 planeNormal = new PlaneNormal3D_F64();

		@Override
		public boolean generate(List<Point3D_F64> dataSet, PlaneGeneral3D_F64 output) {
			Point3D_F64 p = dataSet.get(0);

			planeNormal.p = p;

			if( !fit.svdPoint(dataSet, p, planeNormal.n) )
				return false;

			UtilPlane3D_F64.convert(planeNormal,output);

			return true;
		}

		@Override
		public int getMinimumPoints() {
			return 3;
		}
	}
}
