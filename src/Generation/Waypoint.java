package Generation;

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
	
	public Waypoint offset(double xOffset, double yOffset) {
		double newX = x + xOffset; 
		double newY = y + yOffset; 
		return new Waypoint(newX, newY); 
	}
	
	//A method to get a perpendicauliar offset of a point
	/*
	 * @param the offset left is positive, right is negative
	 */
	public Waypoint offsetWaypointPerpen(double offset) {
		double angle = heading + Math.PI / 2;
		Waypoint point = Waypoint.PolarPoint(offset, angle); 
		point = point.offset(x, y); 
		return new Waypoint(point.x, point.y, heading);  
	} 
}
