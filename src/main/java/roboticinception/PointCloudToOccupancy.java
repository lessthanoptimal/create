package roboticinception;

import bubo.maps.d2.grid.GridMapSpacialInfo;
import bubo.maps.d2.grid.OccupancyGrid2D_F32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import org.ddogleg.struct.GrowQueue_F64;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Peter Abeles
 */
public class PointCloudToOccupancy {

	// transform from the camera to local ground reference frame
	Se3_F64 cameraToGround;

	int minPointsCell = 10;

	double ignoreHeight = 1.0;
	double minHeight = -0.01;
	double maxHeight = 0.02;

	GridMapSpacialInfo mapSpacial;
	OccupancyGrid2D_F32 map;

	List<GrowQueue_F64> pointsInCell = new ArrayList<GrowQueue_F64>();

	public PointCloudToOccupancy(Se3_F64 cameraToGround,
								 GridMapSpacialInfo mapSpacial,
								 OccupancyGrid2D_F32 map) {
		this.cameraToGround = cameraToGround;
		this.mapSpacial = mapSpacial;
		this.map = map;

		int N = map.getWidth()*map.getHeight();
		for (int i = 0; i < N; i++) {
			pointsInCell.add( new GrowQueue_F64());
		}
	}

	public void process( List<Point3D_F64> cloud ) {

		map.setAll(0.5f);
		for (int i = 0; i < pointsInCell.size(); i++) {
			pointsInCell.get(i).reset();
		}

		Point2D_F64 worldPt = new Point2D_F64();
		Point2D_F64 mapPt = new Point2D_F64();

		for (int i = 0; i < cloud.size(); i++) {
			Point3D_F64 p = cloud.get(i);

			if( p.z > ignoreHeight )
				continue;

			worldPt.set(p.x, p.y);
			mapSpacial.globalToMap(worldPt,mapPt);

			int cellX = (int)Math.floor(mapPt.x);
			int cellY = (int)Math.floor(mapPt.y);

			if( !map.isInBounds(cellX,cellY))
				continue;

			GrowQueue_F64 list = pointsInCell.get(cellY*map.getWidth() + cellX);
			list.add( p.z);
		}

		for (int i = 0; i < pointsInCell.size(); i++) {
			GrowQueue_F64 l = pointsInCell.get(i);

			if( l.size() < minPointsCell )
				continue;

			double mean = 0;
			for (int j = 0; j < l.size(); j++) {
				mean += l.get(j);
			}
			mean /= l.size;

			int gridX = i%map.getWidth();
			int gridY = i/map.getWidth();

			if( mean < minHeight || mean > maxHeight )
				map.set(gridX,gridY,1.0f);
			else
				map.set(gridX,gridY,0);
		}
	}

	public Se3_F64 getCameraToGround() {
		return cameraToGround;
	}

	public int getMinPointsCell() {
		return minPointsCell;
	}

	public double getIgnoreHeight() {
		return ignoreHeight;
	}

	public double getMinHeight() {
		return minHeight;
	}

	public double getMaxHeight() {
		return maxHeight;
	}

	public GridMapSpacialInfo getMapSpacial() {
		return mapSpacial;
	}

	public OccupancyGrid2D_F32 getMap() {
		return map;
	}
}
