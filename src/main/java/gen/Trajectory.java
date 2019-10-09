package gen;

import gen.segments.CubicBezierSegment.CubicBezierSegmentFactory;
import gen.segments.CubicHermiteSegment.CubicHermiteSegmentFactory;

/**
 * Trajectory.java
 *
 * @author Tahsin Ahmed
 *
 * Uses a spline to generate a trajectory.
 * Includes time, velocity, acceleration, jerk.
 */

import java.util.ArrayList;

public class Trajectory {

	private ArrayList<Waypoint> traj;

	public final int sampleRate;
	public final double dt;
	public final double wheelBaseWidth;
	public final double maxVelocity, maxAcceleration, maxJerk;
	public final FitMethod method;
	public final Spline spline;

	/**
	 * @param method which type of spline to use.
	 * @param sampleRate how many points per segment to use to sample curve.
	 *                   higher rates give better results.
	 * @param tightness a scale factor for heading.
	 *                  affects how robot turns through out the path.
	 * @param dt how far apart each point should be in time.
	 * @param wheelBaseWidth width of robot.
	 * @param maxVelocity maximum allowed velocity of path.
	 * @param maxAcceleration maximum allowed acceleration of path.
	 * @param maxJerk maximum allowed jerk of path.
	 * @param waypoints control points used to generate spline.
	 */
	public Trajectory(FitMethod method, int sampleRate, double tightness, double dt, double wheelBaseWidth, double maxVelocity, double maxAcceleration, double maxJerk, Waypoint... waypoints) {
		this.method = method;
		switch(method) {
			case CUBIC_BEZIER:
				spline = new Spline(tightness, new CubicBezierSegmentFactory(), waypoints);
				break;
			default:
				spline = new Spline(tightness, new CubicHermiteSegmentFactory(), waypoints);
				break;
		}
		this.sampleRate = sampleRate;
		this.dt = dt;
		this.wheelBaseWidth = wheelBaseWidth;
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.maxJerk = maxJerk;
		traj = new ArrayList<>();
	}

	/**
	 * @param value value to be bounded
	 * @param max highest allowed value
	 * @param min lowest allowed value
	 * @return value bounded between min and max
	 */
	private static double bound(double value, double max, double min) {
		return Math.min(Math.max(value, min), max);
	}

	/**
	 * @return waypoints of the trajectory.
	 */
	public ArrayList<Waypoint> getPoints() {
		try {
			ArrayList<Waypoint> clone = new ArrayList<>(traj.size());
			for (Waypoint waypoint : traj) {
				clone.add((Waypoint) waypoint.clone());
			}
			return clone;
		} catch (Exception e) {
			System.out.println("error in copying");
			return null;
		}
	}

	/**
	 * samples the spline by generating sampleRate #
	 * of points per segment.
	 */
	private void sample() {
		if (!traj.isEmpty()) traj.clear();
		int NUMBER_OF_POINTS = sampleRate * spline.size();
		for (int i = 0; i <= NUMBER_OF_POINTS; i++) {
			double a = (double) i / NUMBER_OF_POINTS;
			traj.add(spline.getWaypoint(a));
		}
	}

	/**
	 * generates a trapezodial velocity curve
	 * with maxVelocity as cruise velocity.
	 */
	// adapted from jackfel's (team 3641) white paper which explains how to find velocity found here:
	// https://www.chiefdelphi.com/t/how-does-a-robot-pathfinder-motion-profiler-work/165533
	private void getVelocities() {
		for (Waypoint waypoint : traj) {
			//Gets the velocity for the accelerating, cruise, and decelerating cases
			//Using the kinematic equation Vf^2 = Vi^2 + 2ad
			double accelerate = Math.sqrt(2 * maxAcceleration * waypoint.distanceFromStart);
			double cruise = maxVelocity;
			double decelerate = Math.sqrt(2 * maxAcceleration * waypoint.distanceFromEnd);

			//Sets the velocity to the minimum of these
			waypoint.velocity = Math.min(Math.min(accelerate, cruise), decelerate);
		}
	}


	/**
	 * finds the designated time at each pt
	 * using distance vs. velocity.
	 */
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

	/**
	 * finds accelerations at each point
	 * using velocity vs. time.
	 */
	private void getAccelerations() {
		traj.get(0).acceleration = 0;
		for(int i = 1; i < traj.size(); i++) {
			double velocityDiff = traj.get(i).velocity - traj.get(i - 1).velocity;
			double timeDiff = traj.get(i).time - traj.get(i - 1).time;
			traj.get(i).acceleration = bound(velocityDiff / timeDiff, maxAcceleration, -maxAcceleration);
		}
	}

	/**
	 * finds jerk at each point
	 * using acceleration vs. time.
	 */
	private void getJerks() {
		traj.get(0).jerk = 0;
		for(int i = 1; i < traj.size(); i++) {
			double accelerationDiff = traj.get(i).acceleration - traj.get(i - 1).acceleration;
			double timeDiff = traj.get(i).time - traj.get(i - 1).time;
			traj.get(i).jerk = bound(accelerationDiff / timeDiff, maxJerk, -maxJerk);
		}
	}

	/**
	 * uses a sample trajectory to find specific pts
	 * that are dt apart from each other.
	 */
	private void timeParameterize() {
		double totalTime = traj.get(traj.size() - 1).time;
		// delete the old trajectory and save it elsewhere
		ArrayList<Waypoint> ct = new ArrayList<>(traj);
		traj.clear();

		// iterate through the timestamps adding dt or the time difference each time
		int index = 0;
		for(double time = 0; time <= totalTime; time += dt) {
			for (int i = index; i < ct.size() - 1; i++) {
				// find the pair of points with times around our target time
				if (ct.get(i).time <= time && ct.get(i + 1).time > time) {
					double timeDiff = ct.get(i + 1).time - ct.get(i).time;
					double timeNeed = time - ct.get(i).time;
					double percentNeed = timeNeed / timeDiff;
					double percentage = (i + percentNeed) / (sampleRate * spline.size());
					traj.add(spline.getWaypoint(percentage));
					index = i;
					break;
				}
			}
		}
		calculate();
	}

	/**
	 * fully calculates trajectory from a samples spline.
	 */
	private void calculate() {
		getVelocities();
		getTimes();
		getAccelerations();
		getJerks(); //60 ft/sec^3 best for trapezoidal motion profile
	}

	/**
	 * samples spline and then generates a time parameterized trajectory.
	 */
	public void generate() {
		sample();
		calculate();
		timeParameterize();
	}

	/**
	 * @author Tahsin Ahmed
	 *
	 * Types of ways to generate splines.
	 */
	public enum FitMethod {

		CUBIC_BEZIER("cubic bezier"), CUBIC_HERMITE("cubic hermite");

		private String method;

		/**
		 * @param method name of method.
		 */
		FitMethod(String method) {
			this.method = method;
		}

		/**
		 * @param str name of method in String format.
		 * @return method as an enum.
		 */
		public static FitMethod findMethod(String str) {
			for (FitMethod method : FitMethod.values()) {
				if (method.getMethod().equals(str.toLowerCase())) return method;
			}
			return null;
		}

		public String getMethod() {
			return method;
		}

		@Override
		public String toString() {
			return method;
		}

	}

	/**
	 * @author Tahsin Ahmed
	 *
	 * Some pre-defined values for sample rate of spline.
	 */
    public enum SampleRate {

		LOW(1000), MEDIUM(10000), HIGH(100000);

		private int rate;

		/**
		 * @param rate sample rate
		 */
		SampleRate(int rate) {
			this.rate = rate;
		}

		public int getRate() {
			return rate;
		}

		@Override
		public String toString() {
			return Integer.toString(rate);
		}

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
