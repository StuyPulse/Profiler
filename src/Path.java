import java.util.ArrayList;


public class Path {
	//Trajectory contains all the calculated points while waypoints are just the points the user provided
	//Trajectory has all the points while waypoint only stores the points the user gave in (x, y, and heading) 
	//curvepoints stores the points that define each curve
	ArrayList<Waypoint> centralTrajectory;
	Waypoint[] waypoints;  
	Waypoint[][] curvepoints; 
	int numberOfPoints; //The number of points per curve
	double dt; 
	double totalDistance; 
	double totalTime; 
	double maxVelocity; 
	double maxAcceleration; 
	double maxJerk;
	double wheelBaseWidth; //The offset for the paths is half the wheel base width
	ArrayList<Waypoint> leftTrajectory, rightTrajectory; 
	Waypoint[][] leftCurve, rightCurve;  
	
	//Enter information for the center of the robot 
	public Path(int numberOfPoints, double wheelBaseWidth, double maxVelocity, double maxAcceleration, double maxJerk, Waypoint... waypoints) {
		if(waypoints.length < 2) {
			System.out.println("Not enough points");
			System.exit(0);
		} 
		this.numberOfPoints = numberOfPoints;
		this.wheelBaseWidth = wheelBaseWidth; 
		this.maxVelocity = maxVelocity; 
		this.maxAcceleration = maxAcceleration; 
		this.maxJerk = maxJerk; 
		this.waypoints = waypoints; 
		this.curvepoints = new Waypoint[waypoints.length - 1][4];  
		centralTrajectory = new ArrayList<Waypoint>();
		leftTrajectory = new ArrayList<Waypoint>();
		rightTrajectory = new ArrayList<Waypoint>();
		findCentralTrajectory();
		findSideTrajectories();
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
	private void genBezierPath(int numberOfPoints, double tightness, ArrayList<Waypoint> trajectory, Waypoint... waypoints) {
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
			
			this.curvepoints[i][0] = startwp; 
			this.curvepoints[i][1] = control1.toWaypoint();
			this.curvepoints[i][2] = control2.toWaypoint();
			this.curvepoints[i][3] = endwp;  
			for(int j = 1; j < numberOfPoints; j++) {
				double percentage = (double) j / (double) numberOfPoints;
				Point pathPoint = cubicBezier(startwp, control1, control2, endwp, percentage); 
				trajectory.add(pathPoint.toWaypoint());
			}
			trajectory.add(endwp);
			//The latest added point which is a path point is true
			trajectory.get(trajectory.size() - 1).isPathPoint = true; 
		}
	}

	//Generates a Bezier Path from given curvepoints including the control points
	/*
	 * @param the prespecified curvepoints
	 */
	private void genBezierPath(Waypoint[][] curvepoints, ArrayList<Waypoint> trajectory) { 
		trajectory.add(curvepoints[0][0]); 
		for(int i = 0; i < curvepoints.length; i++) { 
			for(int j = 1; j < numberOfPoints; j++) {
				double percentage = (double) j / (double) numberOfPoints; 
				Point pathPoint = cubicBezier(curvepoints[i][0], curvepoints[i][1], curvepoints[i][2], curvepoints[i][3], percentage); 
				trajectory.add(pathPoint.toWaypoint()); 
			}
			trajectory.add(curvepoints[i][3]);
		}	
	}
	
	//Gets the distances from the start, of each waypoint
	//Requires that all the curvepoints be generated already
	private void getDistancesFromStart(ArrayList<Waypoint> trajectory) {
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
	private void getDistancesFromEnd(ArrayList<Waypoint> trajectory) {
		double totalDistance = trajectory.get(trajectory.size() - 1).distanceFromStart; 
		for(int i = 0; i < trajectory.size(); i++) {
			trajectory.get(i).distanceFromEnd = totalDistance - trajectory.get(i).distanceFromStart;
		}
	}
	
	private void getDistancesFromLast(ArrayList<Waypoint> trajectory) {
		trajectory.get(0).distanceFromLast = 0;  
		for(int i = 1; i < trajectory.size(); i++) {
			trajectory.get(i).distanceFromLast = trajectory.get(i).distanceFromStart - trajectory.get(i - 1).distanceFromStart; 
		}
	}
	//Gets the headings of each waypoint
	//Requires that all the curvepoints be generated already
	private void getHeadings(ArrayList<Waypoint> trajectory) {
		for(int i = 0; i < trajectory.size() - 1; i++) {
			trajectory.get(i).getHeading(trajectory.get(i + 1));
		}
	}
	
	//Gets the velocity and acceleration of the curvepoints under a trapezodial motion profile
	//Requires that the distances be found
	/*
	 * @param the max velocity that the robot should be at cruise 
	 * @param the max acceleration that the robot should be when accelerating
	 */
	private void timeParameterize(ArrayList<Waypoint> trajectory) {
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
	private void getTimes(ArrayList<Waypoint> trajectory) {
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
	private void getJerks(ArrayList<Waypoint> trajectory) {
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

	//Gets the angular velocity at each point
	//Requires that the headings and times be found
	//This is used for velocity correction, and to find the velocities of the side trajectories
	private void getAngularVelocities(ArrayList<Waypoint> trajectory) {
		trajectory.get(0).angularVelocity = 0;   
		for(int i = 1; i < trajectory.size(); i++) {
			double headingDiff = trajectory.get(i).heading - trajectory.get(i - 1).heading; 
			double timeDiff = trajectory.get(i).time - trajectory.get(i - 1).time; 
			trajectory.get(i).angularVelocity = headingDiff / timeDiff;  
		}
	}

	//Finds the trajectory of the center of the robot 
	public void findCentralTrajectory() {
		genBezierPath(numberOfPoints, 0.8, centralTrajectory, waypoints);
		getDistancesFromStart(centralTrajectory); 
		getDistancesFromEnd(centralTrajectory);
		getDistancesFromLast(centralTrajectory); 
		getHeadings(centralTrajectory);
		getAngularVelocities(centralTrajectory);
		timeParameterize(centralTrajectory);
		getTimes(centralTrajectory);
		getJerks(centralTrajectory); //60 ft/sec^3 best for trapezodial motion profile
	}

	//Offsets the points of the curve
	/*
	 * @param the distance that the point should be offseted in the perpendicular orienation
	 */
	private Waypoint[][] getOffsetPathPoints(double offset) {
		if(curvepoints.length < 1) {
			return this.curvepoints; 
		}
		Waypoint[][] curvepoints = new Waypoint[this.curvepoints.length][4];
		double change = 0.1; 
		if(offset < 0) {
			change *= -1;
		}else if(offset == 0) { 
			return this.curvepoints; 
		}
		double accuracy = 0.3;
		Orientation orientation;     
		for(int i = 0; i < curvepoints.length; i++) { 
			orientation = Orientation.findOrientation(this.curvepoints[i][3].heading); 
			for(int j = 0; j < curvepoints[i].length; j ++) {
				if(j == 0) {
					Point point = this.curvepoints[i][j].offsetPerpendicular(this.curvepoints[i][j + 1], offset, change, accuracy, orientation); 
					curvepoints[i][j] = new Waypoint(point.x, point.y, this.curvepoints[i][j].heading);  
				}else if(j == curvepoints[i].length - 1) {
					Point point = this.curvepoints[i][j].offsetPerpendicular(this.curvepoints[i][j - 1], offset, change, accuracy, orientation);
					curvepoints[i][j] = new Waypoint(point.x, point.y, this.curvepoints[i][j].heading);
				}else {
					Point pointOffset1 = this.curvepoints[i][j].offsetPerpendicular(this.curvepoints[i][j - 1], offset, change, accuracy, orientation); 
					Point otherOffset1 = this.curvepoints[i][j - 1].offsetPerpendicular(this.curvepoints[i][j], offset, change, accuracy, orientation);
					Point pointOffset2 = this.curvepoints[i][j].offsetPerpendicular(this.curvepoints[i][j + 1], offset, change, accuracy, orientation);
					Point otherOffset2 = this.curvepoints[i][j + 1].offsetPerpendicular(this.curvepoints[i][j], offset, change, accuracy, orientation);
					double[] equation1 = pointOffset1.linearEquation(otherOffset1); 
					double[] equation2 = pointOffset2.linearEquation(otherOffset2);
					double[] pointCoordinates = MatrixMath.solveLinearSystem(equation1, equation2);
					if(Double.isNaN(pointCoordinates[0]) || Double.isNaN(pointCoordinates[1])) {
						Point point = pointOffset1.avgPoint(pointOffset2);
						if(Double.isNaN(pointCoordinates[0]) && Double.isNaN(pointCoordinates[1])) {
							curvepoints[i][j] = point.toWaypoint(); 
						}else if(Double.isNaN(pointCoordinates[0])) {
							curvepoints[i][j] = new Waypoint(point.x, pointCoordinates[1]); 
						}else {
							curvepoints[i][j] = new Waypoint(pointCoordinates[0], point.y); 
						} 
					}else { 
						curvepoints[i][j] = new Waypoint(pointCoordinates[0], pointCoordinates[1]);
					}
				}
			}
		}
		return curvepoints;  
	}

	private void getSideVelocities(ArrayList<Waypoint> sideTrajectory) {
		for(int i = 0; i < sideTrajectory.size(); i++) {
			double velocityDiff = centralTrajectory.get(i).angularVelocity * (wheelBaseWidth / 2);  
			if(sideTrajectory.get(i).distanceFromLast > centralTrajectory.get(i).distanceFromLast) {
				sideTrajectory.get(i).velocity = centralTrajectory.get(i).velocity + velocityDiff; 
			}else if(sideTrajectory.get(i).distanceFromLast < centralTrajectory.get(i).distanceFromLast) {
				sideTrajectory.get(i).velocity = centralTrajectory.get(i).velocity - velocityDiff;
			}else {
				sideTrajectory.get(i).velocity = centralTrajectory.get(i).velocity; 
			}
		}
	}

	//Finds the trajectories for the sides of the robot 
	public void findSideTrajectories() {
		double offset = wheelBaseWidth / 2; 
		leftCurve = getOffsetPathPoints(-offset); rightCurve = getOffsetPathPoints(offset);
		genBezierPath(leftCurve, leftTrajectory); genBezierPath(rightCurve, rightTrajectory); 
		getDistancesFromStart(leftTrajectory); getDistancesFromStart(rightTrajectory);
		getDistancesFromEnd(leftTrajectory); getDistancesFromEnd(rightTrajectory);
		getHeadings(leftTrajectory); getHeadings(rightTrajectory); 
		//getSideVelocities(leftTrajectory); getSideVelocities(rightTrajectory);
	}
}