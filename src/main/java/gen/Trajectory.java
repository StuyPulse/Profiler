package gen;

import gen.splines.CubicBezier;
import gen.splines.CubicHermite;
import gen.splines.Spline;

import java.util.ArrayList;
import java.util.Arrays;

public class Trajectory {
	private ArrayList<Waypoint> traj;
	private Waypoint[] waypoints;
	private int sampleRate;
	private double tightness;
	private double dt;
	private double wheelBaseWidth;
	private double maxVel,
				     maxAccel,
					 maxJerk;
	private Spline spline;

	public enum FitMethod {
		CUBIC_BEZIER, CUBIC_HERMITE;

		public static FitMethod getMethod(String method) {
			switch (method.toLowerCase()) {
				case "cubic bezier":
					return FitMethod.CUBIC_BEZIER;
				default:
					return FitMethod.CUBIC_HERMITE;
			}
		}
	}

	public Trajectory(FitMethod method, int sampleRate, double tightness, double dt, double wheelBaseWidth, double maxVel, double maxAccel, double maxJerk, Waypoint... waypoints) {
		spline = null; 
		switch(method) {
			case CUBIC_BEZIER:
				spline = new CubicBezier(); 
				break;
			case CUBIC_HERMITE:
				spline = new CubicHermite();  
				break;  
		}
		this.sampleRate = sampleRate; 
		this.tightness = tightness; 
		this.dt = dt; 
		this.wheelBaseWidth = wheelBaseWidth;
		this.maxVel = maxVel; 
		this.maxAccel = maxAccel; 
		this.maxJerk = maxJerk; 
		traj = new ArrayList<>();
		this.waypoints = waypoints;
	}

	public ArrayList<Waypoint> getPath() {
		return traj;
	}
	
	private void genPath() {
		traj = spline.getPath(sampleRate, spline.getSplines(tightness, waypoints));
	}

	//Gets the distances from the start, of each waypoint
	//Requires that all the points be generated already
	private void getDistancesFromStart() {
		/*double totalDistance = 0;
		traj.get(0).distanceFromStart = totalDistance;
		for(int i = 1; i < traj.size(); i++) {
			double distance = traj.get(i).distanceTo(traj.get(i - 1));
			totalDistance += distance;
			traj.get(i).distanceFromStart = totalDistance;
		}*/
		double distance = 0;
		traj.get(0).distanceFromStart = distance;
		Waypoint[] lastSpline = traj.get(0).spline;
		for(int i = 1; i < traj.size(); i++) {
			double travelled = spline.integrate(0, traj.get(i).alpha, traj.get(i).spline);
			if(!Arrays.equals(traj.get(i).spline, lastSpline)) {
				distance += spline.integrate(0, 1, lastSpline);
			}
			travelled += distance;
			traj.get(i).distanceFromStart = travelled;
		}
	}
	
	//Gets the distances from the end, of each waypoint
	//Requires that the distances from the start be calculated
	private void getDistancesFromEnd() {
		double totalDistance = traj.get(traj.size() - 1).distanceFromStart;
		for(Waypoint point : traj) {
			point.distanceFromEnd = totalDistance - point.distanceFromStart;
		}
	}
	
	//Gets the headings of each waypoint counterclockwise in radians
	//Requires that all the points be generated already
	private void getHeadings() {
		for(Waypoint point : traj) {
			//find the tangents (or derivatives)
			Waypoint tangent = spline.differentiate(point.alpha, point.spline);
			//heading = atan2(dy, dx)
			point.heading = Math.atan2(tangent.y, tangent.x);
			//atan2 only returns from the range -PI to PI, so here is a fix to get pos values
			point.heading = (2 * Math.PI + point.heading) % (2 * Math.PI);
		}
	}
	
	//Gets the velocity and acceleration of the curvepoints under a trapezodial motion profile for the central path
	//Requires that the distances be found
	//To do velocity corrections angular velocities must be found
	/*
	 * @param if velocity should be restrained or not at turns
	 */
	//adapted from jackfel's (team 3641) white paper which explains how to find velocity found here: 
	//https://www.chiefdelphi.com/t/how-does-a-robot-pathfinder-motion-profiler-work/165533
	private void getVelocities() {
		for (Waypoint waypoint : traj) {
			//Gets the velocity for the accelerating, cruise, and decelerating cases
			//Using the kinematic equation Vf^2 = Vi^2 + 2ad
			double accelVel = Math.sqrt(2 * maxAccel * waypoint.distanceFromStart);
			double cruiseVel = maxVel;
			double decelVel = Math.sqrt(2 * maxAccel * waypoint.distanceFromEnd);

			//Sets the velocity to the minimum of these
			double lowest = Math.min(Math.min(accelVel, cruiseVel), decelVel);
			if (lowest == accelVel) {
				waypoint.velocity = accelVel;
			} else if (lowest == cruiseVel) {
				waypoint.velocity = cruiseVel;
			} else {
				waypoint.velocity = decelVel;
			}
		}
	}

	//Gets the time by which it will take the robot to be at that point
	//Requires that the distances and velocities be found
	private void getTimes() {
		double totalTime = 0;
		traj.get(0).time = totalTime;
		for(int i = 1; i < traj.size(); i++) {
			double dd = traj.get(i).distanceFromStart - traj.get(i-1).distanceFromStart;
			double dv = traj.get(i).velocity - traj.get(i-1).velocity;
			double minVel = Math.min(traj.get(i).velocity, traj.get(i-1).velocity);
			//derived from theorem that area under velocity curve equals distance
			double dt = dd / (minVel + Math.abs(dv) / 2);
			totalTime += dt;
			traj.get(i).time = totalTime;
		}
	}

	//Gets the accelerations for the central path
	//Requires that velocities and times be found
	private void getAccelerations() {
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
	private void getJerks() {
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
	
	/*
	//Gets the angular velocity at each point
	//Requires that the headings and times be found
	protected void getAngularVelocities() {
		traj.get(0).angularVelocity = 0;   
		for(int i = 1; i < traj.size(); i++) {
			double headingDiff = traj.get(i).heading - traj.get(i - 1).heading; 
			double timeDiff = traj.get(i).time - traj.get(i - 1).time; 
			traj.get(i).angularVelocity = headingDiff / timeDiff;
		}
	}
	
	//Ensures that all velocities are under the max velocity cap
	private void correctVelocities() {
		getAngularVelocities();
		for(int i = 0; i < traj.size(); i++) {
			//calculate the outside wheel velocity
			double velDiff = Math.abs(traj.get(i).angularVelocity) * (wheelBaseWidth / 2); 
			double outside = traj.get(i).velocity + velDiff;
			if(outside > maxVel) {
				//lower the velocity but keep the same proportion
				//thank you to adrisj (PoSE 2017-18 of 694) for the idea 
				traj.get(i).velocity = traj.get(i).velocity * maxVel / outside; 
			}
		}
		getTimes();
		getAccelerations();
		getJerks();
	}
	*/

	//Time Parameterize the trajectory such that points are a certain time away
	private void timeParameterize() {
		double totalTime = traj.get(traj.size() - 1).time;
		//Delete the old trajectory and save it elsewhere 
		ArrayList<Waypoint> copy = new ArrayList<Waypoint>(traj);
		traj.clear();

		int index = 0;
		//Iterate through the timestamps adding dt or the time difference each time   
		for(double time = 0; time <= totalTime; time += dt) {
			for (int i = index; i < copy.size() - 1; i++) {
				//Find the pair of points with times around our target time
				if (copy.get(i).time <= time && copy.get(i + 1).time > time) {
					double timeDifference = copy.get(i + 1).time - copy.get(i).time;
					double timeNeeded = time - copy.get(i).time;
					double percentNeeded = timeNeeded / timeDifference;
					double percentage = percentNeeded / sampleRate + copy.get(i).alpha;
					traj.add(spline.getWaypoint(percentage, copy.get(i).spline));
					index = i;
					break;
				}
			}
		}
		calculate();
	}

	private void calculate() {
		//Path
		getDistancesFromStart();
		getDistancesFromEnd();
		getHeadings();

		//Trajectory
		getVelocities();
		getTimes();
		getAccelerations();
		getJerks(); //60 ft/sec^3 best for trapezoidal motion profile
	}

	public void generate() {
		genPath();
		calculate();
		timeParameterize();
	}

	/*
	private SideTraj offsetTraj(double offsetCartesian) {
		return new SideTraj(this, offsetCartesian);
	}
	
	public SideTraj getLeft() {
		return offsetTraj(wheelBaseWidth / 2);
	}
	
	public SideTraj getRight() {
		return offsetTraj(-wheelBaseWidth / 2);
	}
	*/
}
