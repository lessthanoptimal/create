package roboticinception.log;

import bubo.io.text.ReadCsv;
import roboticinception.CreateOdometry;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.List;

/**
 * Reads in a create odometry log file
 *
 * @author Peter Abeles
 */
public class CreateOdometryParser {

	ReadCsv reader;

	CreateOdometry odometry = new CreateOdometry();
	boolean next;

	public CreateOdometryParser(String fileName) {
		try {
			reader = new ReadCsv(new FileInputStream(fileName));
			reader.setComment('#');
			parseLine();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public boolean hasNext() {
		return next;
	}

	public void next( CreateOdometry odometry ) throws IOException
	{
		odometry.set(this.odometry);
		parseLine();
	}

	private void parseLine() throws IOException {
		List<String> words = reader.extractWords();

		if( words == null ) {
			next = false;
		} else {
			if( words.size() != 5 )
				throw new IOException("Unexpected number of words. Expected 5, found "+words.size());

			odometry.time  = Long.parseLong(words.get(0));
			odometry.x     = Double.parseDouble(words.get(1));
			odometry.y     = Double.parseDouble(words.get(2));
			odometry.theta = Double.parseDouble(words.get(3));
			odometry.gyro  = Integer.parseInt(words.get(4));

//			System.out.println("odometry "+odometry);

			next = true;
		}
	}
}
