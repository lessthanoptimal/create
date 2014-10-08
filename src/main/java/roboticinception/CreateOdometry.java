package roboticinception;

/**
 * @author Peter Abeles
 */
public class CreateOdometry {
	public long time;
	public double x;
	public double y;
	public double theta;
	public int gyro;

	public CreateOdometry(long time, double x, double y, double theta, int gyro) {
		this.time = time;
		this.x = x;
		this.y = y;
		this.theta = theta;
		this.gyro = gyro;
	}

	public CreateOdometry(CreateOdometry original) {
		set(original);
	}

	public CreateOdometry() {
	}

	public void set( CreateOdometry odometry ) {
		time = odometry.time;
		x = odometry.x;
		y = odometry.y;
		theta = odometry.theta;
		gyro = odometry.gyro;
	}

	public long getTime() {
		return time;
	}

	public void setTime(long time) {
		this.time = time;
	}

	public double getX() {
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double getTheta() {
		return theta;
	}

	public void setTheta(double theta) {
		this.theta = theta;
	}

	public int getGyro() {
		return gyro;
	}

	public void setGyro(int gyro) {
		this.gyro = gyro;
	}

	public CreateOdometry copy() {
		return new CreateOdometry(this);
	}

	@Override
	public String toString() {
		return "CreateOdometry{" +
				"time=" + time +
				", x=" + x +
				", y=" + y +
				", theta=" + theta +
				", gyro=" + gyro +
				'}';
	}
}
