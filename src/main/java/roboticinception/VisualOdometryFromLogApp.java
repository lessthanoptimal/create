/*
 * Copyright (c) 2011-2013, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package roboticinception;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.abst.sfm.d3.DepthVisualOdometry;
import boofcv.abst.sfm.d3.VisualOdometry;
import boofcv.alg.distort.DoNothingPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
import boofcv.factory.sfm.FactoryVisualOdometry;
import boofcv.io.MediaManager;
import boofcv.io.UtilIO;
import boofcv.io.image.UtilImageIO;
import boofcv.io.wrapper.DefaultMediaManager;
import boofcv.openkinect.UtilOpenKinect;
import boofcv.struct.calib.VisualDepthParameters;
import boofcv.struct.image.ImageSInt16;
import boofcv.struct.image.ImageUInt16;
import boofcv.struct.image.ImageUInt8;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Bare bones example showing how to estimate the camera's ego-motion using a depth camera system, e.g. Kinect.
 * Additional information on the scene can be optionally extracted from the algorithm if it implements AccessPointTracks3D.
 *
 * @author Peter Abeles
 */
public class VisualOdometryFromLogApp {

	public static void main( String args[] ) throws IOException {

		MediaManager media = DefaultMediaManager.INSTANCE;

		String directory = "log/07-06-2014-19:21:56/";

		// load camera description and the video sequence
		VisualDepthParameters param = UtilIO.loadXML(media.openFile("data/intrinsic_kinect.xml"));

		// specify how the image features are going to be tracked
		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<ImageUInt8> tracker =
				FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(600, 3, 1),
						ImageUInt8.class, ImageSInt16.class);

		DepthSparse3D<ImageUInt16> sparseDepth = new DepthSparse3D.I<ImageUInt16>(1e-3);

		// declares the algorithm
		DepthVisualOdometry<ImageUInt8,ImageUInt16> visualOdometry =
				FactoryVisualOdometry.depthDepthPnP(1.5, 120, 2, 200, 50, true,
						sparseDepth, tracker, ImageUInt8.class, ImageUInt16.class);

		// Pass in intrinsic/extrinsic calibration.  This can be changed in the future.
		visualOdometry.setCalibration(param.visualParam,new DoNothingPixelTransform_F32());

		// TODO need to synchronize image some how

		List<String> images = findMatching(directory,".*ppm");
		List<String> depths = findMatching(directory,".*depth");

		int N = Math.min(images.size(),depths.size());
		ImageUInt16 depth = new ImageUInt16(1,1);
		for (int i = 0; i < N; i++) {

			ImageUInt8 visual = UtilImageIO.loadImage(images.get(i),ImageUInt8.class);
			UtilOpenKinect.parseDepth(depths.get(i),depth,null);

			if( !visualOdometry.process(visual,depth) ) {
				throw new RuntimeException("VO Failed!");
			}

			Se3_F64 leftToWorld = visualOdometry.getCameraToWorld();
			Vector3D_F64 T = leftToWorld.getT();

			System.out.printf("%03d  Location %8.2f %8.2f %8.2f      inliers %s\n",i, T.x, T.y, T.z, inlierPercent(visualOdometry));
		}
	}

	public static List<String> findMatching( String directory , final String regex ) {
		File[] files = new File(directory).listFiles(new FilenameFilter() {
			@Override
			public boolean accept(File dir, String name) {
				return name.matches(regex);
			}
		});

		List<String> out = new ArrayList<>();
		for( File f : files ) {
			out.add(f.getAbsolutePath());
		}
		Collections.sort(out);
		return out;
	}

	/**
	 * If the algorithm implements AccessPointTracks3D, then count the number of inlier features
	 * and return a string.
	 */
	public static String inlierPercent(VisualOdometry alg) {
		if( !(alg instanceof AccessPointTracks3D))
			return "";

		AccessPointTracks3D access = (AccessPointTracks3D)alg;

		int count = 0;
		int N = access.getAllTracks().size();
		for( int i = 0; i < N; i++ ) {
			if( access.isInlier(i) )
				count++;
		}

		return String.format("%%%5.3f", 100.0 * count / N);
	}
}
