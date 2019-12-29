package gen;

/**
 * @author Tahsin Ahmed
 * A waypoint is a point that is part of the trajectory.
 */
public class Waypoint extends Vector {

	//Universal units so longs as you are consistent
	/** The 'time' it should take to travel to the waypoint in. */
	public double time;
	/** Self Explanatory. */
	public double distanceFromStart, 
				  distanceFromEnd;
	/** Self Explanatory. */
	public double velocity, 
				  acceleration, 
				  jerk;
	/** Direction of the waypoint, it is counterclockwise and in radians */
	public double heading;
	public double curvature;

	/**
	 * @param x the x component (vector form).
	 * @param y the y component (vector form).
	 * @param heading the direction, it is counterclockwise and in radians.
	 */
	public Waypoint(double x, double y, double heading) {
		super(x, y);
		this.heading = heading;
	}

	/**
	 * @param v vector form.
	 * @param heading the direction, it is counterclockwise and in radians.
	 */
	public Waypoint(Vector v, double heading) {
		super(v.x, v.y);
		this.heading = heading;
	}

	/**
	 * @param xOffset how much you want to offset the x component by.
	 * @param yOffset how much you want to offset the y component by.
	 * @return offset waypoint.
	 */
	@Override
	public Waypoint offset(double xOffset, double yOffset) {
		return new Waypoint(x + xOffset, y + yOffset, heading);
	}

	/**
	 * @param v offset as a vector
	 * @return offset waypoint
	 */
	@Override
	public Waypoint offset(Vector v) {
		return offset(v.x, v.y);
	}

	/**
	 * @param theta angle in radians to rotate by
	 * @return rotated waypoint
	 */
	public Waypoint rotate(double theta) {
		return new Waypoint(x, y, heading + theta);
	}

	/** Converts a waypoint into a vector. */
	public Vector toVector() {
		return new Vector(x, y);
	}

	/**
	 * @return heading as a vector
	 */
	public Vector direction() {
		return new Vector(Math.cos(heading), Math.sin(heading));
	}

	@Override
	public String toString() {
		return "(" + x + ", " + y + ", " + heading + ")";
	}

}
