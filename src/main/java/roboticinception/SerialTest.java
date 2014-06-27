package roboticinception;

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

// TODO get front bumper
// TODO get wheel encoders
// TODO get battery level
// TODO Cliff.  (left,right) + front (left,right)
public class SerialTest
{
   SerialPort serialPort;
   InputStream in;
   OutputStream out;

   byte[] dataOut = new byte[ 1024 ];
   byte[] dataIn = new byte[ 1024 ];

   {
      // -Djava.library.path=".:libs/mfz-rxtx-2.2-20081207-linux-x86_64/"
//      System.setProperty("java.library.path",".:libs/mfz-rxtx-2.2-20081207-linux-x86_64/");
//      System.loadLibrary("librxtxSerial.so");
   }

   void connect( String portName ) throws Exception {
      System.out.println("Opening port "+portName);

      CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
      CommPort commPort = portIdentifier.open("FOO", 2000);
      serialPort = (SerialPort) commPort;
      serialPort.setSerialPortParams(57600, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
      serialPort.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);

      in = serialPort.getInputStream();
      out = serialPort.getOutputStream();
   }

   public void startTheRobot() {
      // send the start command to the robot
      dataOut[0] = (byte)128;
      send(dataOut,0,1);

      // request full control over the robot
      dataOut[0] = (byte)132;
      send(dataOut,0,1);
   }

   public void sendRobotPassive() {
      // sending the start command puts the robot into passive mode
      dataOut[0] = (byte)128;
      send(dataOut,0,1);
   }

   public void shutdown() {
      try
      {
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
   public void sendDrive( int velocity , int radius) {
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
   public void sendWheelVelocity( int left , int right ) {
      short sleft = (short)left;
      short sright = (short)right;

      dataOut[0] = (byte)145;
      dataOut[1] = (byte)((sright >> 8) & 0xFF);
      dataOut[2] = (byte)(sright& 0xFF);
      dataOut[3] = (byte)((sleft >> 8) & 0xFF);
      dataOut[4] = (byte)(sleft& 0xFF);

      send(dataOut,0,5);
   }

   protected void send( byte[] data , int start , int length ) {
      try
      {
         out.write(data,start,length);
      } catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void printData() {
      System.out.println("Printing input data");
      try {
         if( in.available() > 0 )
         {
            int len = in.read(dataIn);
            if (len > 0)
            {
               System.out.println("Got data! " + len);
//            System.out.print( "data in size = "+len+" data = "+new String( dataIn, 0, len ) );
            }
         } else {
            System.out.println("No data available!");
         }
      } catch( IOException e ) {
         e.printStackTrace();
      }
   }

//   public static class SerialReader implements Runnable {
//
//      InputStream in;
//
//      public SerialReader( InputStream in ) {
//         this.in = in;
//      }
//
//      public void run() {
//         byte[] buffer = new byte[ 1024 ];
//         int len = -1;
//         try {
//            while( ( len = this.in.read( buffer ) ) > -1 ) {
//               System.out.print( new String( buffer, 0, len ) );
//            }
//         } catch( IOException e ) {
//            e.printStackTrace();
//         }
//      }
//   }

   public static void main( String[] args ) {
      try {
         SerialTest serial = new SerialTest();
         serial.connect("/dev/ttyUSB0");
         serial.startTheRobot();
//         serial.sendStartDemo(0);
         serial.sendWheelVelocity(300,-300);
         Thread.sleep(10000);
         serial.sendWheelVelocity(0,0);
//         serial.sendRobotPassive();
//         serial.printData();
         serial.shutdown();
         System.out.println("Done");
         System.exit(0);
      } catch( Exception e ) {
         e.printStackTrace();
      }
   }
}
