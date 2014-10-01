/*
 * Copyright (c) 2011-2014, Peter Abeles. All Rights Reserved.
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

import boofcv.core.image.ConvertBufferedImage;
import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import boofcv.gui.image.VisualizeImageData;
import boofcv.openkinect.UtilOpenKinect;
import boofcv.struct.image.ImageUInt16;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;
import com.sun.jna.NativeLibrary;
import org.openkinect.freenect.*;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

/**
 * Example demonstrating how to process and display data from the Kinect.
 *
 * @author Peter Abeles
 */
public class CreateKinectTestApp {

	{
		NativeLibrary.addSearchPath("freenect", "/home/pja/projects/thirdparty/libfreenect/build/lib");
	}

	MultiSpectral<ImageUInt8> rgb = new MultiSpectral<ImageUInt8>(ImageUInt8.class,1,1,3);
	ImageUInt16 depth = new ImageUInt16(1,1);

	BufferedImage outRgb;
	ImagePanel guiRgb;

	BufferedImage outDepth;
	ImagePanel guiDepth;

	public void process() throws Exception {
		// initialize the create
		CreateDriver serial = new CreateDriver("/dev/ttyUSB1");
		serial.startTheRobot();

		serial.sendKinect(true);

		System.out.println("Pausing for a bit...");
		for (int i = 0; i < 10; i++) {
			System.out.print("*");
			Thread.sleep(1000);
		}
		System.out.println();
		System.out.println("Displaying kinect data");

		Context kinect = Freenect.createContext();

		if( kinect.numDevices() < 0 )
			throw new RuntimeException("No kinect found!");

		Device device;
		try {
			device = kinect.openDevice(0);
		} catch( RuntimeException e ) {
			e.printStackTrace();
			serial.sendKinect(false);
			serial.shutdown();
			return;
		}
		device.setDepthFormat(DepthFormat.REGISTERED);
		device.setVideoFormat(VideoFormat.RGB);

		device.startDepth(new DepthHandler() {
			@Override
			public void onFrameReceived(FrameMode mode, ByteBuffer frame, int timestamp) {
				processDepth(mode,frame,timestamp);
			}
		});
		device.startVideo(new VideoHandler() {
			@Override
			public void onFrameReceived(FrameMode mode, ByteBuffer frame, int timestamp) {
				processRgb(mode,frame,timestamp);
			}
		});

		System.out.println("Streaming for a bit...");
		long starTime = System.currentTimeMillis();
		while( starTime+5000 > System.currentTimeMillis() ) {Thread.yield();}
		System.out.println("Shutting down");

		device.stopDepth();
		device.stopVideo();
		device.close();
		serial.sendKinect(false);
		serial.shutdown();
		System.out.println("Done!");
	}

	protected void processDepth( FrameMode mode, ByteBuffer frame, int timestamp ) {
		System.out.println("Got depth! "+timestamp);

		if( outDepth == null ) {
			depth.reshape(mode.getWidth(),mode.getHeight());
			outDepth = new BufferedImage(depth.width,depth.height, BufferedImage.TYPE_INT_RGB);
			guiDepth = ShowImages.showWindow(outDepth, "Depth Image");
		}

		UtilOpenKinect.bufferDepthToU16(frame, depth);

//		VisualizeImageData.grayUnsigned(depth,outDepth,UtilOpenKinect.FREENECT_DEPTH_MM_MAX_VALUE);
		VisualizeImageData.disparity(depth, outDepth, 0, UtilOpenKinect.FREENECT_DEPTH_MM_MAX_VALUE, 0);
		guiDepth.repaint();
	}

	protected void processRgb( FrameMode mode, ByteBuffer frame, int timestamp ) {
		if( mode.getVideoFormat() != VideoFormat.RGB ) {
			System.out.println("Bad rgb format!");
		}

		System.out.println("Got rgb!   "+timestamp);

		if( outRgb == null ) {
			rgb.reshape(mode.getWidth(),mode.getHeight());
			outRgb = new BufferedImage(rgb.width,rgb.height, BufferedImage.TYPE_INT_RGB);
			guiRgb = ShowImages.showWindow(outRgb, "RGB Image");
		}

		UtilOpenKinect.bufferRgbToMsU8(frame, rgb);
		ConvertBufferedImage.convertTo_U8(rgb, outRgb, true);

		guiRgb.repaint();
	}

	public static void main( String args[] ) throws Exception {
		CreateKinectTestApp app = new CreateKinectTestApp();

		app.process();
	}
}
