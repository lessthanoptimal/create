package roboticinception;

import boofcv.alg.depth.VisualDepthOps;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.UtilImageIO;
import boofcv.openkinect.UtilOpenKinect;
import boofcv.struct.FastQueueArray_I32;
import boofcv.struct.calib.VisualDepthParameters;
import boofcv.struct.image.ImageUInt16;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;
import bubo.gui.maps.GridMapInteractionDisplay;
import bubo.maps.d2.grid.GridMapSpacialInfo;
import bubo.maps.d2.grid.OccupancyGrid2D_F32;
import bubo.maps.d2.grid.impl.ArrayGrid2D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.struct.FastQueue;

import java.io.IOException;

/**
 * @author Peter Abeles
 */
public class OccupancySingleImageApp {
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
			p.scale(1.0/1000.0);
		}

		// load the transform to ground plane plane
		Se3_F64 cameraToGround = UtilIO.loadXML("cameraToGround.xml");

		GridMapSpacialInfo mapSpacial = new GridMapSpacialInfo(0.1,3,0);
		OccupancyGrid2D_F32 map = new ArrayGrid2D_F32(40,40);
		PointCloudToOccupancy generatorMap = new PointCloudToOccupancy(cameraToGround,mapSpacial,map);

		generatorMap.process(cloud.toList());

		GridMapInteractionDisplay gui = new GridMapInteractionDisplay();
		gui.setMap(mapSpacial,map);

		ShowImages.showWindow(gui,"Occupancy Grid");
	}
}
