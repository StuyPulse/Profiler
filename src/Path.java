import java.util.ArrayList;

public class Path {
	ArrayList<Waypoint> trajectory;
	
	public Path(int numberOfPoints, Waypoint... waypoints) {
		trajectory = new ArrayList<Waypoint>(); 
		genBezierPath(numberOfPoints, 0.8, waypoints);
		getDistances(); 
		getHeadings(); 
	}
	
	//Finds a point by using cubic bezier
	/*
	 * @param the first point on the line
	 * @param the first control point 
	 * @param the second control point 
	 * @param the second point on the line
	 * @param the percentage of the way on the curve
	 * @param the percentage of the way on the curve 
	 */
	Point cubicBezier(Point point1, Point control1, Point control2, Point point2, double alpha) {
		double x = point1.x * Math.pow(1 - alpha, 3) + control1.x * 3 * Math.pow(1 - alpha, 2) * alpha + 
					control2.x * 3 * (1 - alpha) * Math.pow(alpha, 2) + point2.x * Math.pow(alpha, 3);
		double y = point1.y * Math.pow(1 - alpha, 3) + control1.y * 3 * Math.pow(1 - alpha, 2) * alpha + 
					control2.y * 3 * (1 - alpha) * Math.pow(alpha, 2) + point2.y * Math.pow(alpha, 3);
		return new Point(x, y); 
	}
	
	//Finds all the points on the path
	/*
	 * @param the number of points between each pair of waypoints
	 * @param the tightness of the curve
	 * @param the waypoints from which the path with be generated
	 */
	void genBezierPath(int numberOfPoints, double tightness, Waypoint... waypoints) {
		trajectory.add(waypoints[0]);
		for(int i = 0; i < waypoints.length - 1; i++) {
			Waypoint startwp = waypoints[i]; 
			Waypoint endwp = waypoints[i + 1];
			double distance = startwp.distanceTo(endwp);
			double gplength = distance / 2 * tightness; 
			Point startOffset = Point.PolarPoint(gplength, startwp.heading);
			Point endOffset = Point.PolarPoint(-gplength, endwp.heading);
			Point control1 = startwp.offset(startOffset.x, startOffset.y);
			Point control2 = endwp.offset(endOffset.x, endOffset.y);
			
			for(int j = 1; j <= numberOfPoints; j++) {
				double percentage = (double) j / (double) numberOfPoints;
				Point pathPoint = cubicBezier(startwp, control1, control2, endwp, percentage); 
				trajectory.add(new Waypoint(pathPoint.x, pathPoint.y));
			}
		}
	}
	
	//Gets the distances from the start of each waypoint
	//Requires that all the waypoints be generated already
	void getDistances() {
		double distanceAccumlator = 0; 
		trajectory.get(0).distance = distanceAccumlator;
		for(int i = 1; i < trajectory.size(); i++) {
			double distance = trajectory.get(i).distanceTo(trajectory.get(i - 1));
			distanceAccumlator += distance; 
			trajectory.get(i).distance = distanceAccumlator; 
		}
	}
	
	//Gets the headings of each point counterclockwise and in radians
	//Requires that all the waypoints be generated already
	void getHeadings() {
		for(int i = 1; i < trajectory.size(); i++) {
			double headingDiff = trajectory.get(i - 1).angleBetween(trajectory.get(i));
			trajectory.get(i).heading = headingDiff; 
		}
	}
}
