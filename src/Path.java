import java.util.ArrayList;


public class Path {
	//Trajectory contains all the calculated points while waypoints are just the points the user provided
	//Notice that the waypoints of the path cannot be changed after they are set
	ArrayList<Waypoint> trajectory; 
	double totalDistance; 
	double totalTime; 
	double maxVelocity; 
	double maxAcceleration; 
	double maxJerk;  
	
	public Path(int numberOfPoints, double maxVelocity, double maxAcceleration, double maxJerk, Waypoint... waypoints) { 
		this.maxVelocity = maxVelocity; 
		this.maxAcceleration = maxAcceleration; 
		this.maxJerk = maxJerk; 
		trajectory = new ArrayList<Waypoint>(); 
		genBezierPath(numberOfPoints, 0.8, waypoints);
		getDistancesFromStart(); 
		getDistancesFromEnd(); 
		getHeadings();
		timeParameterize(this.maxVelocity, this.maxAcceleration);
		getTimes();
		getJerks(this.maxJerk); //60 ft/sec^3 best for trapezodial motion profile
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
		trajectory.get(0).isPathPoint = true; 
		for(int i = 0; i < waypoints.length - 1; i++) {
			Waypoint startwp = waypoints[i]; 
			Waypoint endwp = waypoints[i + 1];
			double distance = startwp.distanceTo(endwp);
			double gplength = distance / 2 * tightness; 
			Point startOffset = Point.PolarPoint(gplength, startwp.heading);
			Point endOffset = Point.PolarPoint(-gplength, endwp.heading);
			Point control1 = startwp.offset(startOffset.x, startOffset.y);
			Point control2 = endwp.offset(endOffset.x, endOffset.y);
			
			for(int j = 1; j < numberOfPoints; j++) {
				double percentage = (double) j / (double) numberOfPoints;
				Point pathPoint = cubicBezier(startwp, control1, control2, endwp, percentage); 
				trajectory.add(new Waypoint(pathPoint.x, pathPoint.y));
			}
			trajectory.add(endwp);
			//The latest added point which is a path point is true
			trajectory.get(trajectory.size() - 1).isPathPoint = true; 
		}
	}
	
	//Gets the distances from the start, of each waypoint
	//Requires that all the waypoints be generated already
	void getDistancesFromStart() {
		double distanceAccumlator = 0; 
		trajectory.get(0).distanceFromStart = distanceAccumlator;
		for(int i = 1; i < trajectory.size(); i++) {
			double distance = trajectory.get(i).distanceTo(trajectory.get(i - 1));
			distanceAccumlator += distance; 
			trajectory.get(i).distanceFromStart = distanceAccumlator; 
		}
		totalDistance = trajectory.get(trajectory.size() - 1).distanceFromStart; 
	}
	
	//Gets the distances from the end, of each waypoint
	//Requires that the distances from the start be calculated
	void getDistancesFromEnd() {
		double totalDistance = trajectory.get(trajectory.size() - 1).distanceFromStart; 
		for(int i = 0; i < trajectory.size(); i++) {
			trajectory.get(i).distanceFromEnd = totalDistance - trajectory.get(i).distanceFromStart;
		}
	}
	
	//Gets the headings of each waypoint
	//Requires that all the waypoints be generated already
	void getHeadings() {
		for(int i = 0; i < trajectory.size() - 1; i++) {
			trajectory.get(i).getHeading(trajectory.get(i + 1));
		}
	}
	
	//Gets the velocity and acceleration of the waypoints under a trapezodial motion profile
	//Requires that the distances be found
	/*
	 * @param the max velocity that the robot should be at cruise 
	 * @param the max acceleration that the robot should be when accelerating
	 */
	void timeParameterize(double maxVelocity, double maxAcceleration) {
		for(int i = 0; i < trajectory.size(); i++) {
			//Gets the velocity for the accelerating, cruise, and decelerating cases
			//Using the kinematic equation Vf^2 = Vi^2 + 2ad
			double accelVel = Math.sqrt(2 * maxAcceleration * trajectory.get(i).distanceFromStart);
			double cruiseVel = maxVelocity; 
			double decelVel = Math.sqrt(2 * maxAcceleration * trajectory.get(i).distanceFromEnd);
			
			//Sets the velocity to the minium of these
			if(accelVel < cruiseVel && accelVel < decelVel) {
				trajectory.get(i).velocity = accelVel;  
				trajectory.get(i).acceleration = maxAcceleration; 
			}else if(cruiseVel < accelVel && cruiseVel < decelVel) {
				trajectory.get(i).velocity = cruiseVel;  
				trajectory.get(i).acceleration = 0; 
			}else if(decelVel < accelVel && decelVel < cruiseVel) {
				trajectory.get(i).velocity = decelVel;
				trajectory.get(i).acceleration = -maxAcceleration; 
			}
		}
	}
	
	//Gets the time by which it will take the robot to be at that point
	//Requires that the distances and velocities be found
	void getTimes() {
		double timeAccumlator = 0; 
		trajectory.get(0).time = timeAccumlator; 
		for(int i = 1; i < trajectory.size(); i++) {
			double distanceTraveled = trajectory.get(i).distanceFromStart - trajectory.get(i - 1).distanceFromStart;
			double avgVelocity = (trajectory.get(i).velocity + trajectory.get(i - 1).velocity) / 2; 
			double timeElapsed = distanceTraveled / avgVelocity;
			timeAccumlator += timeElapsed; 
			trajectory.get(i).time = timeAccumlator; 
		}
		totalTime = trajectory.get(trajectory.size() - 1).time; 
	}
	
	//Gets the jerk of each waypoint
	//Requires that accelerations and times be calculated
	void getJerks(double maxJerk) {
		for(int i = 0; i < trajectory.size() - 1; i++) {
			double accelerationChange = trajectory.get(i + 1).acceleration - trajectory.get(i).acceleration; 
			double timeElapsed = trajectory.get(i + 1).time - trajectory.get(i).time; 
			double jerk = accelerationChange / timeElapsed; 
			if(jerk > maxJerk) { 
				jerk = maxJerk; 
			}else if(jerk < -maxJerk) {
				jerk = -maxJerk; 
			}
			trajectory.get(i).jerk = jerk; 
		}
		trajectory.get(trajectory.size() - 1).jerk = 0; 
	}
}
