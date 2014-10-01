package roboticinception.autocalib;

import bubo.log.LogMotion2;
import georegression.struct.se.Se2_F64;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * @author Peter Abeles
 */
public class TestCalibrateFromTranslation {
	@Test
	public void perfect() {

		LogMotion2 odometry = createLog(0.1,2,10,0.05);
		LogMotion2 sensor = createLog(0.9,2.5,10.5,0.2);

		CalibrateFromTranslation alg = new CalibrateFromTranslation(200,0.05,0.05);

		assertTrue(alg.process(odometry, sensor));

		Se2_F64 found = alg.getFound();
		assertEquals(-0.8,found.getYaw(),1e-8);
	}

	@Test
	public void noise() {

		LogMotion2 odometry = createLog(0.1,2,10,0.05);
		LogMotion2 sensor = createLog(0.9,2.5,10.5,0.2);

		// add some crappy observations
		odometry.getHistory().get(4).motion.set(2,3,0.4);
		sensor.getHistory().get(10).motion.set(1, 2, -0.4);
		sensor.getHistory().get(17).motion.set(2,3,0);

		CalibrateFromTranslation alg = new CalibrateFromTranslation(200,0.0,0.05);

		assertTrue(alg.process(odometry, sensor));

		Se2_F64 found = alg.getFound();
		assertEquals(-0.8,found.getYaw(),1e-8);
	}

	public LogMotion2 createLog( double angle , double startTime , double endTime , double period ) {

		double c = Math.cos(angle);
		double s = Math.sin(angle);

		LogMotion2 log = new LogMotion2();

		double vel = 0.3;

		for( double time = startTime; time < endTime; time += period ) {
			double x = c*time*vel;
			double y = s*time*vel;

			log.add(time,new Se2_F64(x,y,0));
		}

		return log;
	}
}