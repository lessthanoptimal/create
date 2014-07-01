package roboticinception;

import boofcv.io.image.UtilImageIO;
import boofcv.openkinect.UtilOpenKinect;
import boofcv.struct.image.ImageType;
import boofcv.struct.image.ImageUInt16;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;
import org.ddogleg.struct.GrowQueue_I8;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

/**
 * @author Peter Abeles
 */
public class TurtleBotLogger {
	public String directory;

	PrintStream odometryOut;

	LoggerRgb loggerRgb;
	LoggerDepth loggerDepth;

	public TurtleBotLogger() throws FileNotFoundException {
		if( !new File("log").exists() )  {
			throw new RuntimeException("Can't find log directory");
		}

		DateFormat df = new SimpleDateFormat("MM-dd-yyyy-HH:mm:ss");
		Date today = Calendar.getInstance().getTime();
		String reportDate = df.format(today);

		directory = "log/"+reportDate+"/";

		if( !new File(directory).mkdir() )
			throw new RuntimeException("Couldn't create directory");

		odometryOut = new PrintStream(directory+"log_odometry.txt");
		odometryOut.println("# (time) x y theta gyro");

		loggerRgb = new LoggerRgb(directory,10);
		loggerDepth = new LoggerDepth(directory,10);

		loggerRgb.start();
		loggerDepth.start();
	}

	public void addOdometry( long time , double x, double y , double theta , int gyro ) {
		odometryOut.printf("%d %f %f %f %d\n", time, x, y, theta, gyro);
		odometryOut.flush();
	}

	public void addKinectRgb( long timeSystem, long timeSensor , MultiSpectral<ImageUInt8> image ) {
		loggerRgb.addImage(timeSystem, timeSensor, image);
	}

	public void addKinectDepth( long timeSystem, long timeSensor , ImageUInt16 image ) {
		loggerDepth.addImage(timeSystem,timeSensor, image);
	}

	public void stop() {
		loggerRgb.setDone(true);
		loggerDepth.setDone(true);
		odometryOut.close();
	}

	public static class LoggerRgb extends ImageLoggerThread<MultiSpectral<ImageUInt8>> {

		GrowQueue_I8 temp = new GrowQueue_I8();

		public LoggerRgb(String directory, int queueSize) throws FileNotFoundException {
			super(directory, "rgb", queueSize, ImageType.ms(3, ImageUInt8.class));
		}

		@Override
		protected void saveImage(String name, MultiSpectral<ImageUInt8> image) {
			try {
				UtilImageIO.savePPM(image,name+".ppm",temp);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	public static class LoggerDepth extends ImageLoggerThread<ImageUInt16> {

		GrowQueue_I8 temp = new GrowQueue_I8();

		public LoggerDepth(String directory, int queueSize) throws FileNotFoundException {
			super(directory, "depth", queueSize, ImageType.single(ImageUInt16.class));
		}

		@Override
		protected void saveImage(String name, ImageUInt16 image) {
			try {
				UtilOpenKinect.saveDepth(image, name+".depth", temp);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
}
