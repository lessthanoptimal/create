package roboticinception.autocalib;

import bubo.log.LogMotion2;
import georegression.struct.se.Se2_F64;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * @author Peter Abeles
 */
public class TestCalibrateFromMotionGivenRotation {

	@Test
	public void isEnoughMotion() {
		CalibrateFromMotionGivenRotation alg = new CalibrateFromMotionGivenRotation(100,0.1,0.15);

		assertFalse(alg.isEnoughMotion(new Se2_F64(0.1,0,0.1)));
		assertTrue(alg.isEnoughMotion(new Se2_F64(0.3, 0, 0.3)));
	}

	@Test
	public void perfect() {

		checkPerfect(new Se2_F64(0.0,0.0,0.1));
		checkPerfect(new Se2_F64(0.1,0.2,0.0));
		checkPerfect(new Se2_F64(0.1,0.2,0.1));
	}

	private void checkPerfect(Se2_F64 fromAtoB) {

		Se2_F64 fromBtoA = fromAtoB.invert(null);

		LogMotion2 logSensorA = createLog(new Se2_F64(0,0,0),2,20,0.05);
		LogMotion2 logSensorB = createLog(fromAtoB,2,20,0.05);

		CalibrateFromMotionGivenRotation alg = new CalibrateFromMotionGivenRotation(200,fromBtoA.getYaw(),0.06);

		assertTrue(alg.process(logSensorA, logSensorB));

		Se2_F64 foundBtoA= alg.getTransformSensorBtoA();
		assertEquals(fromBtoA.getX(),foundBtoA.getX(),1e-8);
		assertEquals(fromBtoA.getY(),foundBtoA.getY(),1e-8);
		assertEquals(fromBtoA.getYaw(),foundBtoA.getYaw(),1e-8);
	}

	public LogMotion2 createLog( Se2_F64 robotToSensor , double startTime , double endTime , double period ) {

		LogMotion2 log = new LogMotion2();

		double vel = 0.1;
		double velAngle = 0.05;

		Se2_F64 robot0To1 = new Se2_F64(period*vel,0,velAngle*period);
		Se2_F64 worldToRobot0 = new Se2_F64();

		Se2_F64 worldToSensor0 = worldToRobot0.concat(robotToSensor,null);

		for( double time = startTime; time < endTime; time += period ) {

			Se2_F64 worldToRobot1 = worldToRobot0.concat(robot0To1,null);
			Se2_F64 worldToSensor1 = worldToRobot1.concat(robotToSensor,null);

			Se2_F64 sensorNto0 = worldToSensor1.invert(null).concat(worldToSensor0,null);

			log.add(time, sensorNto0);
			worldToRobot0.set(worldToRobot1);
		}

		return log;
	}

}