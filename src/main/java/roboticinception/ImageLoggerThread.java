package roboticinception;

import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageType;

import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

/**
 * A thread which buffers image images for saving.  Can better handle temporary slow down when writing to a harddrive
 * and won't grind other processes to a halt.
 *
 * @author Peter Abeles
 */
public abstract class ImageLoggerThread<T extends ImageBase> extends Thread {

	// if true then it should stop the thread
	volatile boolean done = false;

	// Circular buffer for storing images
	final List<ImageInfo<T>> queue = new ArrayList<ImageInfo<T>>();
	int start =0;
	int size=0;

	// total images saved to disk
	int totalImages = 0;
	// output for time-stamp log
	PrintStream logTime;
	String directory;
	String prefix;

	public ImageLoggerThread( String directory , String prefix , int queueSize , ImageType<T> imageType )
			throws FileNotFoundException
	{
		this.directory = directory;
		this.prefix = prefix;

		logTime = new PrintStream(directory+"log_"+prefix+".txt");
		logTime.println("# (time system) (time sensor) (image number)");
		for (int i = 0; i < queueSize; i++) {
			queue.add( new ImageInfo<T>(imageType.createImage(1,1)));
		}
	}

	/**
	 * Adds an image to the queue to be saved.  If the queue has grown too large then an error message is printed
	 * @param timeSystem Time when the image was received.
	 * @param timeSensor Time stamp from the sensor.
	 * @param image The image which is to be saved.
	 */
	public void addImage( long timeSystem , long timeSensor, T image ) {
		synchronized ( queue ) {
			if( size >= queue.size())  {
				System.err.println("Discarding image");
			}

			// get the next unused image
			int index = (start + size)%queue.size();

			ImageInfo<T> info = queue.get(index);
			info.timeSystem  = timeSystem;
			info.timeSensor = timeSensor;
			info.image.reshape(image.width, image.height);
			info.image.setTo(image);

			size++;
		}
	}

	public void setDone(boolean done) {
		this.done = done;
	}

	@Override
	public void run() {
		while( !done ) {
			synchronized ( queue ) {
				while( size > 1 ) {
					ImageInfo<T> info = queue.get(start);
					start = (start+1)%queue.size();
					size--;

					logTime.println(info.timeSystem+" "+info.timeSensor +" "+totalImages);
					logTime.flush();
					String name = String.format("%s%s%06d",directory,prefix,totalImages);
					saveImage(name, info.image);
					totalImages++;
					Thread.yield();
				}
			}
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {}
		}
		logTime.close();
	}

	protected abstract void saveImage( String name , T image );

	public static class ImageInfo<T extends ImageBase>
	{
		public T image;
		public long timeSystem;
		public long timeSensor;

		public ImageInfo(T image) {
			this.image = image;
		}
	}
}
