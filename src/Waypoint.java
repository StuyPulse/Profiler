
public class Waypoint extends Point {
	double time; //In seconds
	double distance; //From the start point
	double velocity; 
	double acceleration; 
	double heading; //Counterclockwise and in radians 
	
	public Waypoint(double x, double y) {
		super(x, y); 
	}
	
	//A waypoint in the path
	//Use when declaring the waypoint
	/*
	 * @param x coordinate
	 * @param y coordinate
	 * @param heading counterclockwise in radians
	 */
	public Waypoint(double x, double y, double heading) {
		super(x, y);
		this.heading = heading; 
	}
	
	//Finds the change in heading between the a point and another and returns in radians
	/*
	 * @param another waypoint to find the angle between
	 */
	public double angleBetween(Waypoint waypoint) {
		if(this.x == waypoint.x || this.y == waypoint.y) {
			//Straight line case
			return 0; 
		}else {
			double xDiff = waypoint.x - this.x; 
			double yDiff = waypoint.y - this.y; 
			
			if((xDiff > 0 && yDiff > 0) || (xDiff < 0 && yDiff < 0)) {
				//Going counterclockwise
				return Math.atan(Math.abs(yDiff) / Math.abs(xDiff)); 
			}else if((xDiff > 0 && yDiff < 0) || (xDiff < 0 && yDiff > 0)) {
				//Going clockwise
				return -Math.atan(Math.abs(yDiff) / Math.abs(xDiff));
			}else {
				//Not moving
				return 0;
			}
		}
	}
}
