package roboticinception;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * @author Peter Abeles
 */
public class TestCreateDriver
{
	public static boolean called = false;

	@Test
	public void parseData() {
		byte[] data = new byte[]{19,5,29,2,25,13,0,(byte)163};

		called = false;
		CreateDriver driver = new CreateDriver() {
			@Override
			protected void parsePacket( byte[] data , int start , int length) {
				assertEquals(2,start);
				assertEquals(5,length);
				called = true;
			}
		};

		driver.parseData(data,data.length);
		assertTrue(called);
	}

}
