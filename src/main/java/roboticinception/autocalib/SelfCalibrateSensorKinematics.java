package roboticinception.autocalib;

import bubo.log.LogMotion2;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Peter Abeles
 */
// todo add linear motion
// TODO
public class SelfCalibrateSensorKinematics {

	Motion parentMotion;
	List<Motion> childMotions = new ArrayList<Motion>();

	CalibrateFromTranslation calibrate = new CalibrateFromTranslation(300,0.05,0.6);


	public <Data> void setParent(EstimateMotion2D<Data> motion, Data linear, Data rotating) {
		parentMotion = new Motion(motion,linear,rotating);
	}

	public <Data> void addChild( EstimateMotion2D<Data> motion , Data linear , Data rotating ) {
		childMotions.add(new Motion(motion,linear,rotating));
	}

	public void estimateFromLinear() {
		parentMotion.motion.setDataLog(parentMotion.dataLinear);
		LogMotion2 parent = parentMotion.motion.computeNoHint();

		for (int i = 0; i < childMotions.size(); i++) {
			Motion m = childMotions.get(i);
			m.motion.setDataLog(m.dataLinear);
			LogMotion2 child = m.motion.computeNoHint();

			if( calibrate.process(parent,child) ) {
				calibrate.getTransformSensorBtoA().print();
			} else {
				System.out.println("Failed to calibrate a child");
			}
		}
	}

	private static class Motion {
		EstimateMotion2D motion;
		Object dataLinear;
		Object dataRotating;

		private Motion(EstimateMotion2D motion, Object dataLinear, Object dataRotating) {
			this.motion = motion;
			this.dataLinear = dataLinear;
			this.dataRotating = dataRotating;
		}
	}
}
