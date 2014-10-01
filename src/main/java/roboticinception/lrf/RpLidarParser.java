package roboticinception.lrf;

import bubo.io.text.ReadCsv;
import roboticinception.rplidar.RpLidarScan;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.List;

/**
 * Displays RP-LIDAR measurements from a log file
 *
 * @author Peter Abeles
 */
public class RpLidarParser {

	ReadCsv reader;

	RpLidarScan scan = new RpLidarScan();
	boolean next;

	public RpLidarParser(String fileName) {
		try {
			reader = new ReadCsv(new FileInputStream(fileName));
			reader.setComment('#');
			parseScan();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public boolean hasNext() {
		return next;
	}

	public void next( RpLidarScan scan ) throws IOException
	{
		scan.set(this.scan);
		parseScan();
	}

	private void parseScan() throws IOException {
		List<String> words = reader.extractWords();

		scan.reset();
		if( words == null ) {
			next = false;
		} else {
			int N = words.size();
			if( (N-1)%4 != 0 )
				throw new RuntimeException("Unexpected number of words. modulus");

			int count = Integer.parseInt(words.get(0));

			if( count != N/4 )
				throw new RuntimeException("Unexpected number of words. total");

			for (int i = 1; i < N; i += 4) {
				long time = Long.parseLong(words.get(i));
				int angle = Integer.parseInt(words.get(i+1));
				int quality = Integer.parseInt(words.get(i+2));
				int distance = Integer.parseInt(words.get(i+3));

				scan.used.add(angle);
				scan.time[angle] = time;
				scan.quality[angle] = quality;
				scan.distance[angle] = distance;
			}
			next = true;
		}
	}


}
