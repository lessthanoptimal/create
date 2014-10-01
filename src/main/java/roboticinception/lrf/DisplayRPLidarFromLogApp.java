package roboticinception.lrf;

import boofcv.gui.image.ShowImages;
import bubo.gui.sensors.laser2d.LadarComponent;
import georegression.metric.UtilAngle;
import roboticinception.rplidar.RpLidarScan;

import javax.swing.*;
import java.awt.*;

/**
 * @author Peter Abeles
 */
public class DisplayRPLidarFromLogApp {


	public static void main(String[] args) throws Exception {

		RpLidarParser parser = new RpLidarParser("/home/pja/projects/create/log/09-26-2014-16:59:51/log_rplidar.txt");

		final LadarComponent gui = new LadarComponent();
		gui.configure(0, UtilAngle.degreeToRadian(1.0 / 64.0), 5.0, RpLidarScan.N);
		gui.setAutoRescale(true);
		gui.setPreferredSize(new Dimension(400, 400));
		gui.setMinimumSize(gui.getPreferredSize());

		ShowImages.showWindow(gui,"LIDAR Scan");


		final double meters[] = new double[ RpLidarScan.N ];
		RpLidarScan scan = new RpLidarScan();
		int scanNumber = 0;
		while( parser.hasNext() ) {

			parser.next(scan);

			scan.convertMeters(meters);

			SwingUtilities.invokeLater(new Runnable() {
				@Override
				public void run() {
					gui.setData(meters);
					gui.repaint();
				}
			});

			long start = scan.time[ scan.used.get(0)];
			long end = scan.time[ scan.used.get(scan.used.size()-1)];

			System.out.println("Parsing "+scanNumber+++"  period "+(end-start));

			Thread.sleep(end-start);
		}
	}
}
