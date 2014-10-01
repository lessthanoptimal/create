package roboticinception.lrf;

import boofcv.gui.image.ShowImages;
import bubo.desc.sensors.lrf2d.Lrf2dParam;
import bubo.gui.sensors.laser2d.LadarComponent;
import roboticinception.rplidar.RpLidarHighLevelDriver;
import roboticinception.rplidar.RpLidarScan;

import javax.swing.*;
import java.awt.*;

/**
 * @author Peter Abeles
 */
public class DisplayRPLidarHardwareApp {


	public static void main(String[] args) throws Exception {
		RpLidarHighLevelDriver driver = new RpLidarHighLevelDriver();

		driver.initialize("/dev/ttyUSB0",0);

		RpLidarScan scan = new RpLidarScan();

		final LadarComponent gui = new LadarComponent();
		Lrf2dParam param = UtilRpLidar.createParam();

		gui.configure(0, param.getSweepAngle()/param.getNumberOfScans(), 1, param.getNumberOfScans());
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
