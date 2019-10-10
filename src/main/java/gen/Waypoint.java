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
	 * @param distance the radius from the origin.
	 * @param rotation the direction from the origin.
	 * @return waypoint represented in polar coordinates.
	 */
	public static Waypoint PolarPoint(double distance, double rotation) {
		Vector v = Vector.PolarPoint(distance, rotation);
		return new Waypoint(v.x, v.y, rotation);
	}

	/**
	 * @param xOffset how much you want to offset the x component by.
	 * @param yOffset how much you want to offset the y component by.
	 * @return a new waypoint that has been offsetted.
	 */
	public Waypoint offsetCartesian(double xOffset, double yOffset) {
		return new Waypoint(x + xOffset, y + yOffset, heading);
	}

	/**
	 * @param offset the length you wish to offset by.
	 * @param rotation the angle you wish to offset by.
	 * @return a cartesian waypoint thata is offsetted by polar vales.
	 */
	public Waypoint offsetPolar(double offset, double rotation) {
		Waypoint point = Waypoint.PolarPoint(offset, heading + rotation);
		return point.offsetCartesian(x, y);
	}

	/** Converts a waypoint into a vector. */
	public Vector toVector() {
		return new Vector(x, y);
	}

	@Override
	public String toString() {
		return "(" + x + ", " + y + ", " + heading + ")";
	}

}
