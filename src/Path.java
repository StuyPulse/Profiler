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
	double maxVelocity; 
	double maxAcceleration; 
	double maxJerk;
	double wheelBaseWidth; //The offset for the paths is half the wheel base width
	ArrayList<Waypoint> leftTrajectory, rightTrajectory; 
	boolean velCorrection; 
	
	//Enter information for the center of the robot 
	public Path(int numberOfPoints, double dt, double wheelBaseWidth, double maxVelocity, double maxAcceleration, double maxJerk, boolean velCorrection, Waypoint... waypoints) {
		if(waypoints.length < 2) {
			System.out.println("Not enough points");
			System.exit(0);
		} 
		this.numberOfPoints = numberOfPoints;
		this.dt = dt; 
		this.wheelBaseWidth = wheelBaseWidth; 
		this.maxVelocity = maxVelocity; 
		this.maxAcceleration = maxAcceleration; 
		this.maxJerk = maxJerk; 
		this.velCorrection = velCorrection; 
		this.waypoints = waypoints; 
		this.curvepoints = new Waypoint[waypoints.length - 1][4];  
		centralTrajectory = new ArrayList<Waypoint>();
		leftTrajectory = new ArrayList<Waypoint>(); rightTrajectory = new ArrayList<Waypoint>();
		genBezierPath(numberOfPoints, 0.8, centralTrajectory, curvepoints, waypoints);
		findCentral();
		findSides();
		parameterizeTrajectories();
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
	Waypoint cubicBezier(Waypoint point1, Waypoint control1, Waypoint control2, Waypoint point2, double alpha) {
		double x = point1.x * Math.pow(1 - alpha, 3) + control1.x * 3 * Math.pow(1 - alpha, 2) * alpha + 
					control2.x * 3 * (1 - alpha) * Math.pow(alpha, 2) + point2.x * Math.pow(alpha, 3);
		double y = point1.y * Math.pow(1 - alpha, 3) + control1.y * 3 * Math.pow(1 - alpha, 2) * alpha + 
					control2.y * 3 * (1 - alpha) * Math.pow(alpha, 2) + point2.y * Math.pow(alpha, 3);
		return new Waypoint(x, y); 
	}
	
	//Finds all the points on the path
	/*
	 * @param the number of points between each pair of waypoints
	 * @param the tightness of the curve
	 * @param the trajectory on which to load the points
	 * @param the array on which to load the curvepoints
	 * @param the waypoints from which the path with be generated
	 */
	private void genBezierPath(int numberOfPoints, double tightness, ArrayList<Waypoint> trajectory, Waypoint[][] curvepoints, Waypoint... waypoints) {
		trajectory.add(waypoints[0]);
		for(int i = 0; i < waypoints.length - 1; i++) {
			Waypoint startwp = waypoints[i]; 
			Waypoint endwp = waypoints[i + 1];
			double distance = startwp.distanceTo(endwp);
			double gplength = distance / 2 * tightness; 
			Waypoint startOffset = Waypoint.PolarPoint(gplength, startwp.heading);
			Waypoint endOffset = Waypoint.PolarPoint(-gplength, endwp.heading);
			Waypoint control1 = startwp.offset(startOffset.x, startOffset.y);
			Waypoint control2 = endwp.offset(endOffset.x, endOffset.y);
			
			curvepoints[i][0] = startwp; 
			curvepoints[i][1] = control1;
			curvepoints[i][2] = control2;
			curvepoints[i][3] = endwp;  
			for(int j = 1; j < numberOfPoints; j++) {
				double percentage = (double) j / (double) numberOfPoints;
				Waypoint pathPoint = cubicBezier(startwp, control1, control2, endwp, percentage); 
				trajectory.add(pathPoint);
			}
			trajectory.add(endwp);
			//The latest added point which is a path point is true
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
	}
	
	//Gets the distances from the end, of each waypoint
	//Requires that the distances from the start be calculated
	private void getDistancesFromEnd(ArrayList<Waypoint> trajectory) {
		double totalDistance = trajectory.get(trajectory.size() - 1).distanceFromStart; 
		for(int i = 0; i < trajectory.size(); i++) {
			trajectory.get(i).distanceFromEnd = totalDistance - trajectory.get(i).distanceFromStart;
		}
	}
	
	//Gets the headings of each waypoint counterclockwise in radians
	//Requires that all the curvepoints be generated already
	private void getHeadings(ArrayList<Waypoint> trajectory) {
		for(int i = 0; i < trajectory.size() - 1; i++) {
			trajectory.get(i).heading = Math.atan2(trajectory.get(i + 1).y - trajectory.get(i).y, trajectory.get(i + 1).x - trajectory.get(i).x); 
		}
		trajectory.get(trajectory.size() - 1).heading = waypoints[waypoints.length - 1].heading; 
	}
	
	//Gets the velocity and acceleration of the curvepoints under a trapezodial motion profile for the central path
	//Requires that the distances be found
	//To do velocity corrections angular velocities must be found
	/*
	 * @param if velocity should be restrained or not at turns
	 */
	private void getVelocities(boolean correction) { 
		double maxVelocity = this.maxVelocity; 
		for(int i = 0; i < centralTrajectory.size(); i++) {
			//Does the correction for lowering velocity when turning
			if(correction) {
				//Calculate the outside wheel velocity 
				double velDiff = centralTrajectory.get(i).angularVelocity * (wheelBaseWidth / 2); 
				double outside = centralTrajectory.get(i).velocity + velDiff;
				//If outside is greater replace the max velocity with a restrained one
				if(outside >= this.maxVelocity) {
					maxVelocity = Math.pow(this.maxVelocity, 2) / outside; 
				}else {
					maxVelocity = this.maxVelocity; 
				}
			} 

			//Gets the velocity for the accelerating, cruise, and decelerating cases
			//Using the kinematic equation Vf^2 = Vi^2 + 2ad
			double accelVel = Math.sqrt(2 * maxAcceleration * centralTrajectory.get(i).distanceFromStart);
			double cruiseVel = maxVelocity; 
			double decelVel = Math.sqrt(2 * maxAcceleration * centralTrajectory.get(i).distanceFromEnd);
			
			//Sets the velocity to the minium of these
			if(accelVel < cruiseVel && accelVel < decelVel) {
				centralTrajectory.get(i).velocity = accelVel;  
			}else if(cruiseVel < accelVel && cruiseVel < decelVel) {
				centralTrajectory.get(i).velocity = cruiseVel;  
			}else if(decelVel < accelVel && decelVel < cruiseVel) {
				centralTrajectory.get(i).velocity = decelVel;
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
	}

	//Gets the accelerations for the central path
	//Requires that velocities and times be found
	/*
	 * @param the trajectory to get the accelerations from 
	 * @param wether or not to limit to the max acceleration, use only for central trajectory
	 */
	private void getAccelerations(ArrayList<Waypoint> trajectory, boolean constrict) {
		double velocityDiffI = trajectory.get(1).velocity - trajectory.get(0).velocity;
		double timeDiffI = trajectory.get(1).time - trajectory.get(0).time;  
		trajectory.get(0).acceleration = velocityDiffI / timeDiffI; 
		for(int i = 1; i < trajectory.size(); i++) {
			double velocityDiff = trajectory.get(i).velocity - trajectory.get(i - 1).velocity;
			if(velocityDiff == 0) {
				trajectory.get(i).acceleration = 0; 
				continue; 
			} 
			double timeDiff = trajectory.get(i).time - trajectory.get(i - 1).time; 
			trajectory.get(i).acceleration = velocityDiff / timeDiff;
			if(constrict) {
				if(trajectory.get(i).acceleration > maxAcceleration) {
					trajectory.get(i).acceleration = maxAcceleration; 
				}else if(trajectory.get(i).acceleration < -maxAcceleration) {
					trajectory.get(i).acceleration = -maxAcceleration;
				} 
			}
		} 
	}
	
	//Gets the jerk of each waypoint
	//Requires that accelerations and times be calculated
	/*
	 * @param the trajectory to get the jerks from 
	 * @param wether or not to limit to the max jerk, use only for central trajectory
	 */
	private void getJerks(ArrayList<Waypoint> trajectory, boolean constrict) {
		double accelDiffI = trajectory.get(1).acceleration - trajectory.get(0).acceleration; 
		double timeDiffI = trajectory.get(1).time - trajectory.get(0).time; 
		trajectory.get(0).jerk = accelDiffI / timeDiffI; 
		for(int i = 1; i < trajectory.size(); i++) {
			double accelerationDiff = trajectory.get(i).acceleration - trajectory.get(i - 1).acceleration; 
			if(accelerationDiff == 0) {
				trajectory.get(i).jerk = 0; 
			}
			double timeElapsed = trajectory.get(i).time - trajectory.get(i - 1).time; 
			trajectory.get(i).jerk = accelerationDiff / timeElapsed; 
			if(trajectory.get(i).jerk > maxJerk) { 
				trajectory.get(i).jerk = maxJerk; 
			}else if(trajectory.get(i).jerk < -maxJerk) {
				trajectory.get(i).jerk = -maxJerk; 
			} 
		}
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
	//Paths must be made first
	public void findCentral() {
		getDistancesFromStart(centralTrajectory); 
		getDistancesFromEnd(centralTrajectory); 
		getHeadings(centralTrajectory);
		getVelocities(false);
		getTimes(centralTrajectory); 
		getAccelerations(centralTrajectory, true);  
		getJerks(centralTrajectory, true); //60 ft/sec^3 best for trapezodial motion profile
	}

	//Finds the velocities of the side wheels
	//Requires that the angular velocities of the central trajectory be found
	//And that the side paths be found
	private void getSideVelocities() {
		//Iterating through the points of the trajectory
		for(int i = 0 ; i < centralTrajectory.size(); i ++) {
			//Find the difference the sides have to be to be spinning at that angular velocity
			double velocityDifference = (wheelBaseWidth / 2) * centralTrajectory.get(i).angularVelocity;
			if(leftTrajectory.get(i).distanceFromStart > rightTrajectory.get(i).distanceFromStart) {
				//If left side is outside it is greater
				leftTrajectory.get(i).velocity = centralTrajectory.get(i).velocity + velocityDifference; 
				rightTrajectory.get(i).velocity = centralTrajectory.get(i).velocity - velocityDifference; 
			}else if(rightTrajectory.get(i).distanceFromStart > leftTrajectory.get(i).distanceFromStart) {
				//If right side is outside it is greater
				rightTrajectory.get(i).velocity = centralTrajectory.get(i).velocity + velocityDifference;
				leftTrajectory.get(i).velocity = centralTrajectory.get(i).velocity - velocityDifference; 
			}else {
				//Otherwise its all equal and straight
				leftTrajectory.get(i).velocity = centralTrajectory.get(i).velocity; 
				rightTrajectory.get(i).velocity = centralTrajectory.get(i).velocity; 
			}
		}
	}

	//Finds the trajectories for the sides of the robot 
	//Paths must be made first
	public void findSides() { 
		double offset = wheelBaseWidth / 2;
		leftTrajectory.clear(); rightTrajectory.clear();
		for(int i = 0; i < centralTrajectory.size(); i++) {
			//iterate through central trajectory
			//offset each point
			//copy the time and the heading
			leftTrajectory.add(centralTrajectory.get(i).offsetWaypointPerpen(offset));
			leftTrajectory.get(i).time = centralTrajectory.get(i).time; 
			leftTrajectory.get(i).heading = centralTrajectory.get(i).heading; 
			rightTrajectory.add(centralTrajectory.get(i).offsetWaypointPerpen(-offset));
			rightTrajectory.get(i).time = centralTrajectory.get(i).time; 
			rightTrajectory.get(i).heading = centralTrajectory.get(i).heading;  
		} 
		getDistancesFromStart(leftTrajectory); getDistancesFromStart(rightTrajectory);
		getDistancesFromEnd(leftTrajectory); getDistancesFromEnd(rightTrajectory);
		getAngularVelocities(centralTrajectory);
		getVelocities(velCorrection);
		getSideVelocities();
		getAccelerations(leftTrajectory, false); getAccelerations(rightTrajectory, false);
		getJerks(leftTrajectory, false); getJerks(rightTrajectory, false);   
	}

	//Time Parameterizes the trajectory such that points are a certain time away
	/*
	 * @param the trajectory to time parameterize
	 * @param the curvepoints of the above trajectory
	 */
	private void timeParameterize(ArrayList<Waypoint> trajectory, Waypoint[][] curvepoints) {
		double totalTime = trajectory.get(trajectory.size() - 1).time;
		//Delete the old trajectory and save it elsewhere 
		ArrayList<Waypoint> copy = new ArrayList<Waypoint>(); 
		for(int i = 0; i < trajectory.size(); i++) {
			copy.add(trajectory.get(i)); 
		}
		trajectory.clear();

		int index = 0;  
		double percentage = 0;
		int curve;
		//Iterate through the timestamps adding dt or the time difference each time   
		for(double time = 0; time <= totalTime; time += dt) {
			for(int i = 0; i < copy.size() - 1; i++) {  
				//Find the pair of points with times around our target time
				if(copy.get(i).time <= time && copy.get(i + 1).time > time) {
					index = i; 
					double timeDifference = copy.get(i + 1).time - copy.get(i).time; 
					double timeNeeded = time - copy.get(i).time;
					//Get the (extra) percentage needed in whole number format
					double percentNeeded = timeNeeded / timeDifference;
					//Convert the percentage to decimal   
					percentage = (double) (i + percentNeeded) / (double) numberOfPoints; 
					break;   
				}
			}

			//Filter out the percentage, so its less than 1 in order to avoid extrapolation
			while(percentage >= 1) {
				percentage--; 
			}

			//Find the curve that the new point belongs to
			curve = index / numberOfPoints;   
			//Get the point with bezier curves
			Waypoint point = cubicBezier(curvepoints[curve][0], curvepoints[curve][1], curvepoints[curve][2], curvepoints[curve][3], percentage);
			trajectory.add(point); 
		}
	}

	public void parameterizeTrajectories() { 
		timeParameterize(centralTrajectory, curvepoints);
		findCentral();
		findSides();
	}
}