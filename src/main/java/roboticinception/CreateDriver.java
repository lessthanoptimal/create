package roboticinception;

import georegression.struct.se.Se2_F64;
import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * @author Peter Abeles
 */
// TODO Turn kinect on and off

// TODO get battery level
public class CreateDriver
{
	public static final int PACKET_BUMP_WHEEL = 7;
	public static final int PACKET_WALL = 8;
	public static final int PACKET_CLIFF_LEFT = 9;
	public static final int PACKET_CLIFF_FRONT_LEFT = 10;
	public static final int PACKET_CLIFF_FRONT_RIGHT = 11;
	public static final int PACKET_CLIFF_RIGHT = 12;

	public static final int PACKET_DISTANCE = 19;
	public static final int PACKET_ROTATED = 20;

	SerialPort serialPort;
	InputStream in;
	OutputStream out;

	byte[] dataOut = new byte[ 1024 ];

	byte sensorBumpWheel;
	boolean sensorWall;
	boolean sensorCliffLeft;
	boolean sensorCliffFrontLeft;
	boolean sensorCliffFrontRight;
	boolean sensorCliffRight;

	// odometry
	final Se2_F64 location = new Se2_F64();
	final Se2_F64 recentMotion = new Se2_F64();
	final Se2_F64 temp = new Se2_F64();


	// time the sensor data was last updated
	volatile long sensorUpdatedTime;

	volatile long odometryRequestPeriod = 200;

	ReadCreateDataThread createDataThread;
	OdometryRequestThread odometryThread;

	public boolean verbose = false;

	{
		// -Djava.library.path=".:libs/mfz-rxtx-2.2-20081207-linux-x86_64/"
//      System.setProperty("java.library.path",".:libs/mfz-rxtx-2.2-20081207-linux-x86_64/");
//      System.loadLibrary("librxtxSerial.so");
	}

	public CreateDriver( String portName ) throws Exception {
		System.out.println("Opening port "+portName);

		CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
		CommPort commPort = portIdentifier.open("FOO", 2000);
		serialPort = (SerialPort) commPort;
		serialPort.setSerialPortParams(57600, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
		serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);

		in = serialPort.getInputStream();
		out = serialPort.getOutputStream();

	}

	protected CreateDriver() {

	}

	public synchronized void startTheRobot() {
		if( createDataThread != null )
			throw new RuntimeException("Robot already started!");

		// start the listening task
		createDataThread = new ReadCreateDataThread();
		new Thread(createDataThread,"Create Data").start();
		odometryThread = new OdometryRequestThread();
		new Thread(odometryThread,"Odometry").start();

		// send the start command to the robot
		dataOut[0] = (byte)128;
		send(dataOut,0,1);

		// request full control over the robot
		dataOut[0] = (byte)132;
		send(dataOut,0,1);

		// request that it stream sensor data
		sendSensorRequest();
	}

	public synchronized void sendRobotPassive() {
		// sending the start command puts the robot into passive mode
		dataOut[0] = (byte)128;
		send(dataOut,0,1);
	}

	public synchronized void shutdown() {
		try
		{
			createDataThread.requestStop();
			createDataThread = null;
			odometryThread.requestStop();
			odometryThread = null;

			// make sure the robot isn't moving
			sendWheelVelocity(0,0);
			out.close();

		} catch (IOException e)
		{
			throw new RuntimeException(e);
		}
	}
	/**
	 * Send drive commands using high level circle interface
	 *
	 * @param velocity Velocity -500 to 500 mm/s
	 * @param radius Radius -2000 to 2000 mm
	 */
	public synchronized void sendDrive( int velocity , int radius) {
		short svel = (short)velocity;
		short srad = (short)radius;

		if( srad == 0 ) {
			srad = (short)0x8000;
		}

		dataOut[0] = (byte)137;
		dataOut[1] = (byte)((svel >> 8) & 0xFF);
		dataOut[2] = (byte)(svel& 0xFF);
		dataOut[3] = (byte)((srad >> 8) & 0xFF);
		dataOut[4] = (byte)(srad& 0xFF);

		send(dataOut,0,5);
	}
	/**
	 * Specifies the wheel velocities in [-500 to 500] mm/s
	 * @param left Left wheel velocity in mm/s
	 * @param right Right wheel velocity in mm/s
	 */
	public synchronized void sendWheelVelocity( int left , int right ) {
		short sleft = (short)left;
		short sright = (short)right;

		dataOut[0] = (byte)145;
		dataOut[1] = (byte)((sright >> 8) & 0xFF);
		dataOut[2] = (byte)(sright& 0xFF);
		dataOut[3] = (byte)((sleft >> 8) & 0xFF);
		dataOut[4] = (byte)(sleft& 0xFF);

		send(dataOut,0,5);
	}

	public synchronized void sendSensorRequest() {
		dataOut[0] = (byte)148; //Sensor stream
		dataOut[1] = 8;
		dataOut[2] = PACKET_BUMP_WHEEL;//  bumps and wheel drops
		dataOut[3] = PACKET_WALL;//  Wall
		dataOut[4] = PACKET_CLIFF_LEFT;//  Cliff left
		dataOut[5] = PACKET_CLIFF_FRONT_LEFT;// Cliff front left
		dataOut[6] = PACKET_CLIFF_FRONT_RIGHT;// Cliff front right
		dataOut[7] = PACKET_CLIFF_RIGHT;// Cliff right
		dataOut[8] = PACKET_DISTANCE; // distance traveled
		dataOut[9] = PACKET_ROTATED; // angle rotated
		send(dataOut,0,10);
	}

	public synchronized void sendStreamPauseResume( boolean pause ) {
		dataOut[0] = (byte)150; // Stream pause
		dataOut[1] = (byte)(pause ? 0 : 1);
		send(dataOut,0,2);
	}

	protected synchronized void send( byte[] data , int start , int length ) {
		try
		{
			out.write(data,start,length);
		} catch (IOException e)
		{
			throw new RuntimeException(e);
		}
	}

	protected int parseData( byte[] data , int length ) {

		int offset = 0;
		int dataSize;

		if( verbose ) {
			System.out.println("parseData length = " + length);
			for (int i = 0; i < length; i++) {
				System.out.printf("%3d ", (data[i] & 0xFF));
			}
			System.out.println();
		}

		// search for the first good packet it can find
		while( true ) {
			// see if it has consumed all the data
			if( offset >= length ) {
				return length;
			}
			int type = data[offset]&0xFF;
			if( type == 19 )
			{
				dataSize = data[offset + 1] & 0xFF;
				int packetSize = dataSize+3;
				if( verbose) System.out.println("packet size "+dataSize+"  offset "+offset);
				if ( length < offset+packetSize) {
					if( verbose) System.out.println("  partial packet");
					return offset; // it doesn't have the whole packet yet
				}

				int checksum = 0;
				for (int i = 0; i < packetSize; i++)
				{
					checksum += data[offset + i] & 0xFF;
				}
				if ((checksum & 0xFF) != 0)
				{
					if( verbose) System.out.println("  Bad data: "+checksum);
				} else {
					parsePacket(data,offset+2,dataSize);
				}
				offset += packetSize;
			} else {
				offset++;
			}
		}
	}

	protected void parsePacket( byte[] data , int start , int length) {

		// pause the stream now
		sendStreamPauseResume(true);

		// distance in (mm) and rotation in degrees
		short distanceMM = 0, angleDeg = 0;

		int x = start;
		int end = start + length;
		while (x < end) {
			switch (data[x++] & 0xFF) {
				case PACKET_BUMP_WHEEL:
					sensorBumpWheel = data[x];
					break;

				case PACKET_WALL:
					sensorWall = data[x] != 0;
					break;

				case PACKET_CLIFF_LEFT:
					sensorCliffLeft = data[x] != 0;
					break;

				case PACKET_CLIFF_FRONT_LEFT:
					sensorCliffFrontLeft = data[x] != 0;
					break;

				case PACKET_CLIFF_FRONT_RIGHT:
					sensorCliffFrontRight = data[x] != 0;
					break;

				case PACKET_CLIFF_RIGHT:
					sensorCliffRight = data[x] != 0;
					break;

				case PACKET_DISTANCE:
					distanceMM = (short) ((data[x++] & 0xFF) << 8 | (data[x] & 0xFF));
					break;

				case PACKET_ROTATED:
					angleDeg = (short) ((data[x++] & 0xFF) << 8 | (data[x] & 0xFF));
					break;

				default:
					System.out.println("Unknown packet data type! " + (data[x - 1] & 0xFF));
			}
			x++;
		}

		// update the estimated location
		double distance = distanceMM * 0.001;
		// CCW angles are positive and CW negative
		double theta = angleDeg * Math.PI / 180.0;
		double s = Math.sin(theta);
		double c = Math.cos(theta);

		recentMotion.set(distance * c, distance * s, c, s);
		synchronized (location) {
			location.concat(recentMotion, temp);
			location.set(temp);
		}

		sensorUpdatedTime = System.currentTimeMillis();
	}

	public boolean isWheelDropCaster() {
		return (sensorBumpWheel & 0x10) != 0;
	}

	public boolean isWheelDropLeft() {
		return (sensorBumpWheel & 0x08) != 0;
	}

	public boolean isWheelDropRight() {
		return (sensorBumpWheel & 0x04) != 0;
	}

	public boolean isBumpLeft() {
		return (sensorBumpWheel & 0x02) != 0;
	}

	public boolean isBumpRight() {
		return (sensorBumpWheel & 0x01) != 0;
	}

	public void getLocation( Se2_F64 location ) {
		synchronized (this.location) {
			location.set(this.location);
		}
	}

	public long getSensorUpdatedTime() {
		return sensorUpdatedTime;
	}

	public class OdometryRequestThread implements Runnable {

		volatile boolean run = true;

		public void requestStop() {
			run = false;
		}

		@Override
		public void run() {

			long timeOdometry = 0;

			while( run ) {
				long currentTime = System.currentTimeMillis();

				if( currentTime >= timeOdometry ) {
					timeOdometry = currentTime + odometryRequestPeriod;
					sendStreamPauseResume(false); // resume the stream
				}

				try
				{
					Thread.sleep(Math.max(5,timeOdometry-System.currentTimeMillis()));
				} catch (InterruptedException ignore){}
			}
		}
	}

	public class ReadCreateDataThread implements Runnable {

		byte data[] = new byte[1024*2];
		int size = 0;
		volatile boolean run = true;

		public void requestStop() {
			run = false;
		}

		@Override
		public void run()
		{
			while( run )
			{
				try
				{
					if (in.available() > 0)
					{
						int totalRead = in.read(data,size,data.length-size);

						size += totalRead;

						int used = parseData(data,size);

						// shift the buffer over by the amount read
						for (int i = 0; i < size-used; i++)
						{
							data[i] = data[i+used];
						}
						size -= used;

					}

					Thread.sleep(5);
				} catch (IOException e)
				{
					e.printStackTrace();
				} catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}
		}
	}

	public static void main( String[] args ) {
		try {
			CreateDriver serial = new CreateDriver("/dev/ttyUSB0");
			serial.startTheRobot();
         serial.sendWheelVelocity(200,100);
			long start = System.currentTimeMillis();
			Se2_F64 location = new Se2_F64();
			while( start + 10000 > System.currentTimeMillis() ) {
				serial.getLocation(location);

				System.out.println("bump  left = "+serial.isBumpLeft()+" right "+serial.isBumpRight());
				System.out.println("drop  left = "+serial.isWheelDropLeft()+" right "+serial.isWheelDropRight());
				System.out.println("caster = "+serial.isWheelDropCaster());
				System.out.println("x = "+location.getX()+" y = "+location.getY()+" yaw = "+location.getYaw());

				Thread.sleep(200);
			}
			serial.shutdown();
			System.out.println("Done");
			System.exit(0);
		} catch( Exception e ) {
			e.printStackTrace();
		}
	}
}
