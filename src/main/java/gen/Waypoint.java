package gen;

public class Waypoint {
	//Universal units so longs as you are consistent
	public double x, y; 
	public double time;
	public double distanceFromStart, 
				  distanceFromEnd; 
	public double velocity, 
				  acceleration, 
				  jerk;  
	public double heading, //Counterclockwise and in radians
				  angularVelocity;
	//Spline from which the point is found
	public double alpha;
	public Waypoint[] spline;
	
	public Waypoint(double x, double y) {
		this.x = x; 
		this.y = y; 
	}
	
	//A waypoint in the path
	//Use when declaring the waypoint
	/*
	 * @param x coordinate
	 * @param y coordinate
	 * @param heading counterclockwise in radians
	 */
	public Waypoint(double x, double y, double heading) {
		this.x = x; 
		this.y = y; 
		this.heading = heading; 
	}

	//Polar point
	/*
	 *@param distance form the origin 
	 *@param rotation counterclockwise from the positive x axis in radians 
	*/
	public static Waypoint PolarPoint(double distance, double rotation) {
		double x = Math.cos(rotation) * distance;
		double y = Math.sin(rotation) * distance;
		return new Waypoint(x, y);
	}
	
	public double distanceTo(Waypoint point) {
		return Math.sqrt(Math.pow(point.x - this.x, 2) + Math.pow(point.y - this.y, 2));
	}
	
	public Waypoint offsetCartesian(double xOffset, double yOffset) {
		return new Waypoint(x + xOffset, y+ yOffset);
	}
	
	public Waypoint offsetPolar(double offset, double rotation) {
		Waypoint point = Waypoint.PolarPoint(offset, heading + rotation);
		return point.offsetCartesian(x, y);
	}

	public String toString() {
		return "(" + x + ", " + y + ", " + heading + ")";
	}
}
