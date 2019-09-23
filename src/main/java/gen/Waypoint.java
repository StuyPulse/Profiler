package gen;

import java.util.Objects;

public class Waypoint extends Vector implements Cloneable {

	//Universal units so longs as you are consistent
	public double time;
	public double distanceFromStart, 
				  distanceFromEnd; 
	public double velocity, 
				  acceleration, 
				  jerk;
	public double heading; //Counterclockwise and in radians

	public Waypoint(double x, double y, double heading) {
		super(x, y);
		this.heading = heading;
	}

	public Waypoint(Vector v, double heading) {
		super(v.x, v.y);
		this.heading = heading;
	}

	public static Waypoint PolarPoint(double distance, double rotation) {
		Vector v = Vector.PolarPoint(distance, rotation);
		return new Waypoint(v.x, v.y, rotation);
	}
	
	public Waypoint offsetCartesian(double xOffset, double yOffset) {
		return new Waypoint(x + xOffset, y + yOffset, heading);
	}
	
	public Waypoint offsetPolar(double offset, double rotation) {
		Waypoint point = Waypoint.PolarPoint(offset, heading + rotation);
		return point.offsetCartesian(x, y);
	}

	public Vector toVector() {
		return new Vector(x, y);
	}

	@Override
	public String toString() {
		return "(" + x + ", " + y + ", " + heading + ")";
	}

	@Override
	public boolean equals(Object o) {
		if (!this.getClass().equals(o.getClass())) return false;
		Waypoint w = (Waypoint) o;
		return x == w.x && y == w.y && heading == w.heading &&
				time == w.time &&
				distanceFromStart == w.distanceFromStart && distanceFromEnd == w.distanceFromEnd &&
				velocity == w.velocity && acceleration == w.acceleration && jerk == w.jerk;
	}

	@Override
	public int hashCode() {
		return Objects.hash(x, y, heading,
				time,
				distanceFromStart, distanceFromEnd,
				velocity, acceleration, jerk);
	}

	@Override
	public Object clone() throws CloneNotSupportedException {
		return super.clone();
	}

}
