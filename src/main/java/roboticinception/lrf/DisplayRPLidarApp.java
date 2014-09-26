package roboticinception.lrf;

import boofcv.gui.image.ShowImages;
import bubo.gui.sensors.laser2d.LadarComponent;
import georegression.metric.UtilAngle;
import roboticinception.rplidar.RpLidarHighLevelDriver;
import roboticinception.rplidar.RpLidarScan;

import javax.swing.*;
import java.awt.*;

/**
 * @author Peter Abeles
 */
public class DisplayRPLidarApp {


	public static void main(String[] args) throws Exception {
		RpLidarHighLevelDriver driver = new RpLidarHighLevelDriver();

		driver.initialize("/dev/ttyUSB0",0);

		RpLidarScan scan = new RpLidarScan();

		final LadarComponent gui = new LadarComponent();
		gui.configure(0, UtilAngle.degreeToRadian(1.0 / 64.0), 5.0, RpLidarScan.N);
		gui.setAutoRescale(true);
		gui.setPreferredSize(new Dimension(400, 400));
		gui.setMinimumSize(gui.getPreferredSize());

		ShowImages.showWindow(gui,"LIDAR Scan");



		while( true ) {
			if (!driver.blockCollectScan(scan, 1000)) {
				System.out.println("Couldn't get a complete scan!");
			} else {
				final double meters[] = new double[ RpLidarScan.N ];
				scan.convertMeters(meters);
//				for (int i = 0; i < RpLidarScan.N; i++) {
//					if (scan.distance[i] != 0 && (i/64.0) > 270 ) {
//						System.out.println("distance[" + i + "] = " + scan.distance[i] + "  meters " + meters[i]);
//					}
//				}
				SwingUtilities.invokeLater(new Runnable() {
					@Override
					public void run() {
						gui.setData(meters);
						gui.repaint();
					}
				});
			}
		}

	}
}
