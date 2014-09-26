package roboticinception.moving;

import boofcv.openkinect.UtilOpenKinect;
import boofcv.struct.image.ImageUInt16;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;
import com.sun.jna.NativeLibrary;
import georegression.struct.se.Se2_F64;
import org.openkinect.freenect.*;
import roboticinception.CreateDriver;
import roboticinception.MiscOps;
import roboticinception.TurtleBotLogger;
import roboticinception.rplidar.RpLidarHighLevelDriver;
import roboticinception.rplidar.RpLidarScan;

import java.nio.ByteBuffer;

/**
 * @author Peter Abeles
 */
public class CollectReferenceFrameDataApp {

	TurtleBotLogger logger;
	Device device;
	CreateDriver serial;

	MultiSpectral<ImageUInt8> rgb = new MultiSpectral<ImageUInt8>(ImageUInt8.class,1,1,3);
	ImageUInt16 depth = new ImageUInt16(1,1);
	boolean logImages;
	RpLidarHighLevelDriver lrf = new RpLidarHighLevelDriver();
	boolean logLrf;

	{
		NativeLibrary.addSearchPath("freenect", "/home/pja/projects/thirdparty/libfreenect/build/lib");
	}

	public void init() throws Exception {
		logImages = false;
		logger = new TurtleBotLogger();

		serial = new CreateDriver("/dev/ttyUSB0");
		serial.startTheRobot();
		serial.sendKinect(true);

		System.out.println("Battery = "+serial.getBatteryCharge());

		MiscOps.sleep(5,"Warming up kinect");
		if( !initializeKinect() ) {
			serial.shutdown();
			System.exit(0);
		}
		System.out.println("Battery = "+serial.getBatteryCharge());
		MiscOps.sleep(2,"Waiting for auto shutter to do its thing");
		logImages = true;

		logLrf = true;
		new Thread() {
			@Override
			public void run() {
				RpLidarScan scan = new RpLidarScan();
				if( lrf.initialize("/dev/ttyUSB1",0) ) {

					while (logLrf) {
						if (lrf.blockCollectScan(scan, 300)) {
							logger.addRPLidar(scan);
						}
						Thread.yield();
					}

					lrf.stop();
				} else {
					System.err.println("Can't initialize RP-LIDAR");
				}
			}
		};
	}

	public void perform() {
		long timeStart = System.currentTimeMillis();
		long updateTime = 0;
		Se2_F64 location = new Se2_F64();

		serial.sendWheelVelocity(200, 200);

		while(timeStart+15000 > System.currentTimeMillis() ) {
			if( serial.getSensorUpdatedTime() != updateTime ) {
				updateTime = serial.getSensorUpdatedTime();
				System.out.println("Logging odometry "+updateTime);

				serial.getLocation(location);
				logger.addOdometry(updateTime,location.getX(),location.getY(),location.getYaw(),
						serial.getCargobayAnalog());
			}
		}
	}

	public void shutdown() {
		logLrf = true;
		serial.sendDrive(0,0);
		serial.shutdown();
		logger.stop();
		device.stopDepth();
		device.stopVideo();
		device.close();
	}

	public boolean initializeKinect() {
		Context kinect = Freenect.createContext();

		if( kinect.numDevices() < 0 )
			throw new RuntimeException("No kinect found!");

		try {
			device = kinect.openDevice(0);
		} catch( RuntimeException e ) {
			e.printStackTrace();
			return false;
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

		return true;
	}

	protected void processDepth( FrameMode mode, ByteBuffer frame, int timestamp ) {

		if( !logImages )
			return;

		depth.reshape(mode.getWidth(),mode.getHeight());
		UtilOpenKinect.bufferDepthToU16(frame, depth);

		logger.addKinectDepth(System.currentTimeMillis(),timestamp,depth);
	}

	protected void processRgb( FrameMode mode, ByteBuffer frame, int timestamp ) {
		if( !logImages )
			return;

		if( mode.getVideoFormat() != VideoFormat.RGB ) {
			System.out.println("Bad rgb format!");
		}
		rgb.reshape(mode.getWidth(),mode.getHeight());

		UtilOpenKinect.bufferRgbToMsU8(frame, rgb);
		logger.addKinectRgb(System.currentTimeMillis(),timestamp, rgb);
	}

	public static void main(String[] args) throws Exception {
		CollectReferenceFrameDataApp app = new CollectReferenceFrameDataApp();

		app.init();
		app.perform();
		app.shutdown();
		System.exit(0);
	}
}
