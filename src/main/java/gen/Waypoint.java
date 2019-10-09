package gen;

public class Waypoint extends Vector {

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

}
