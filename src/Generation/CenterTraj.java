package Generation;
import java.util.ArrayList;

public class CenterTraj {
	protected ArrayList<Waypoint> traj; 
	protected Waypoint[] waypoints; 
	protected Waypoint[][] curvepoints;
	protected int sampleRate; 
	protected double dt;
	protected double wheelBaseWidth;
	protected double maxVel, 
				     maxAccel, 
				     maxJerk;
	
	public CenterTraj(int sampleRate, double dt, double wheelBaseWidth, double maxVel, double maxAccel, double maxJerk, Waypoint... waypoints) {
		this.sampleRate = sampleRate; 
		this.dt = dt; 
		this.wheelBaseWidth = wheelBaseWidth;
		this.maxVel = maxVel; 
		this.maxAccel = maxAccel; 
		this.maxJerk = maxJerk; 
		traj = new ArrayList<Waypoint>(); 
		this.waypoints = waypoints; 
		curvepoints = new Waypoint[waypoints.length - 1][4]; 
	}
	
	public ArrayList<Waypoint> getPath() {
		return traj;
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
	protected Waypoint cubicBezier(Waypoint point1, Waypoint control1, Waypoint control2, Waypoint point2, double alpha) {
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
	protected void genBezierPath(double tightness) {
		traj.add(waypoints[0]);
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
			for(int j = 1; j < sampleRate; j++) {
				double percentage = (double) j / (double) sampleRate;
				Waypoint pathPoint = cubicBezier(startwp, control1, control2, endwp, percentage); 
				traj.add(pathPoint);
			}
			traj.add(endwp);
			//The latest added point which is a path point is true
		}
	}
	
	//Gets the distances from the start, of each waypoint
	//Requires that all the curvepoints be generated already
	protected void getDistancesFromStart() {
		double distanceAccumlator = 0; 
		traj.get(0).distanceFromStart = distanceAccumlator;
		for(int i = 1; i < traj.size(); i++) {
			double distance = traj.get(i).distanceTo(traj.get(i - 1));
			distanceAccumlator += distance; 
			traj.get(i).distanceFromStart = distanceAccumlator; 
		}
	}
	
	//Gets the distances from the end, of each waypoint
	//Requires that the distances from the start be calculated
	protected void getDistancesFromEnd() {
		double totalDistance = traj.get(traj.size() - 1).distanceFromStart; 
		for(int i = 0; i < traj.size(); i++) {
			traj.get(i).distanceFromEnd = totalDistance - traj.get(i).distanceFromStart;
		}
	}
	
	//Gets the headings of each waypoint counterclockwise in radians
	//Requires that all the curvepoints be generated already
	protected void getHeadings() {
		for(int i = 0; i < traj.size() - 1; i++) {
			traj.get(i).heading = Math.atan2(traj.get(i + 1).y - traj.get(i).y, traj.get(i + 1).x - traj.get(i).x); 
		}
		traj.get(traj.size() - 1).heading = waypoints[waypoints.length - 1].heading; 
	}
	
	//Gets the velocity and acceleration of the curvepoints under a trapezodial motion profile for the central path
	//Requires that the distances be found
	//To do velocity corrections angular velocities must be found
	/*
	 * @param if velocity should be restrained or not at turns
	 */
	protected void getVelocities() {  
		for(int i = 0; i < traj.size(); i++) { 
			//Gets the velocity for the accelerating, cruise, and decelerating cases
			//Using the kinematic equation Vf^2 = Vi^2 + 2ad
			double accelVel = Math.sqrt(2 * maxAccel * traj.get(i).distanceFromStart);
			double cruiseVel = maxVel; 
			double decelVel = Math.sqrt(2 * maxAccel * traj.get(i).distanceFromEnd);
			
			//Sets the velocity to the minium of these
			double lowest = Math.min(Math.min(accelVel, cruiseVel), decelVel);
			if(lowest == accelVel) {
				traj.get(i).velocity = accelVel;  
			}else if(lowest == cruiseVel) {
				traj.get(i).velocity = cruiseVel;  
			}else {
				traj.get(i).velocity = decelVel;
			} 
		}
	}
	
	//Gets the time by which it will take the robot to be at that point
	//Requires that the distances and velocities be found
	protected void getTimes() {
		double timeAccumlator = 0; 
		traj.get(0).time = timeAccumlator; 
		for(int i = 1; i < traj.size(); i++) {
			double distanceTraveled = traj.get(i).distanceFromStart - traj.get(i - 1).distanceFromStart;
			double avgVelocity = (traj.get(i).velocity + traj.get(i - 1).velocity) / 2; 
			double timeElapsed = distanceTraveled / avgVelocity;
			timeAccumlator += timeElapsed; 
			traj.get(i).time = timeAccumlator; 
		}
	}

	//Gets the accelerations for the central path
	//Requires that velocities and times be found
	/*
	 * @param the trajectory to get the accelerations from 
	 * @param wether or not to limit to the max acceleration, use only for central trajectory
	 */
	protected void getAccelerations() { 
		traj.get(0).acceleration = 0; 
		for(int i = 1; i < traj.size(); i++) {
			double velocityDiff = traj.get(i).velocity - traj.get(i - 1).velocity;
			if(velocityDiff == 0) {
				traj.get(i).acceleration = 0; 
				continue; 
			} 
			double timeDiff = traj.get(i).time - traj.get(i - 1).time; 
			traj.get(i).acceleration = velocityDiff / timeDiff;

			if(traj.get(i).acceleration > maxAccel) {
				traj.get(i).acceleration = maxAccel; 
			}else if(traj.get(i).acceleration < -maxAccel) {
				traj.get(i).acceleration = -maxAccel;
			}
		} 
	}
	
	//Gets the jerk of each waypoint
	//Requires that accelerations and times be calculated
	/*
	 * @param the trajectory to get the jerks from 
	 * @param wether or not to limit to the max jerk, use only for central trajectory
	 */
	protected void getJerks() {
		traj.get(0).jerk = 0; 
		for(int i = 1; i < traj.size(); i++) {
			double accelerationDiff = traj.get(i).acceleration - traj.get(i - 1).acceleration; 
			if(accelerationDiff == 0) {
				traj.get(i).jerk = 0; 
			}
			double timeElapsed = traj.get(i).time - traj.get(i - 1).time; 
			traj.get(i).jerk = accelerationDiff / timeElapsed;
			
			if(traj.get(i).jerk > maxJerk) { 
				traj.get(i).jerk = maxJerk; 
			}else if(traj.get(i).jerk < -maxJerk) {
				traj.get(i).jerk = -maxJerk; 
			} 
		}
	}
	
	
	//Gets the angular velocity at each point
	//Requires that the headings and times be found
	//This is used for velocity correction, and to find the velocities of the side trajectories
	private void getAngularVelocities() {
		traj.get(0).angularVelocity = 0;   
		for(int i = 1; i < traj.size(); i++) {
			double headingDiff = traj.get(i).heading - traj.get(i - 1).heading; 
			double timeDiff = traj.get(i).time - traj.get(i - 1).time; 
			traj.get(i).angularVelocity = headingDiff / timeDiff;
		}
	}
	
	//Ensures that all velocites are under the max velocity cap
	//requires that the velocites and angular velocities be found
	private void correctVelocities() {
		for(int i = 0; i < traj.size(); i++) {
			//calculate the outside wheel velocity
			double velDiff = Math.abs(traj.get(i).angularVelocity) * (wheelBaseWidth / 2); 
			double outside = traj.get(i).velocity + velDiff;
			if(outside > maxVel) {
				//lower the velocity but keep the same proportion
				traj.get(i).velocity = traj.get(i).velocity * maxVel / outside; 
			}
		}
	}

	//Time Parameterizes the trajectory such that points are a certain time away
	/*
	 * @param the trajectory to time parameterize
	 * @param the curvepoints of the above trajectory
	 */
	protected void timeParameterize() {
		double totalTime = traj.get(traj.size() - 1).time;
		//Delete the old trajectory and save it elsewhere 
		ArrayList<Waypoint> copy = new ArrayList<Waypoint>(); 
		for(int i = 0; i < traj.size(); i++) {
			copy.add(traj.get(i)); 
		}
		traj.clear();

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
					percentage = (double) (i + percentNeeded) / (double) sampleRate; 
					break;   
				}
			}

			//Filter out the percentage, so its less than 1 in order to avoid extrapolation
			while(percentage >= 1) {
				percentage--; 
			}

			//Find the curve that the new point belongs to
			curve = index / sampleRate;   
			//Get the point with bezier curves
			Waypoint point = cubicBezier(curvepoints[curve][0], curvepoints[curve][1], curvepoints[curve][2], curvepoints[curve][3], percentage);
			point.time = time; 
			traj.add(point); 
		}
		
		getDistancesFromStart(); 
		getHeadings();
		traj.get(0).velocity = 0; 
		for(int i = 1; i < traj.size(); i++) {
			double distanceTravelled = traj.get(i).distanceFromStart - 
					traj.get(i - 1).distanceFromStart; 
			double timeElapsed = traj.get(i).time - 
					traj.get(i - 1).time; 
			traj.get(i).velocity = distanceTravelled / timeElapsed; 
		}
		getAccelerations(); 
		getJerks();
		getAngularVelocities(); 
	}
	
	public void generate() {
		//Path values
		genBezierPath(0.8);
		getDistancesFromStart(); 
		getDistancesFromEnd(); 
		getHeadings();
				
		//Velocity and velocity corrections
		getVelocities(); 
		getTimes();
		getAngularVelocities();
		correctVelocities(); 
		getTimes();

		//Accelerations and jerk
		getAccelerations();  
		getJerks(); //60 ft/sec^3 best for trapezodial motion profile
		
		//Time parameterizing
		timeParameterize(); 
	}
	
	private SideTraj offsetTraj(double offset) {
		return new SideTraj(this, offset);
	}
	
	public SideTraj getLeft() {
		return offsetTraj(wheelBaseWidth / 2);
	}
	
	public SideTraj getRight() {
		return offsetTraj(-wheelBaseWidth / 2);
	}
}
