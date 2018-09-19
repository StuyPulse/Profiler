
public class Point {
	double x; 
	double y; 
	
	//Regular point
	public Point(double x, double y) {
		this.x = x; 
		this.y = y; 
	}
	
	//Polar point
	/*
	 *@param distance form the origin 
	 *@param rotation counterclockwise from the positive x axis in radians 
	*/
	public static Point PolarPoint(double distance, double rotation) {
		double x = Math.cos(rotation) * distance;
		double y = Math.sin(rotation) * distance;
		return new Point(x, y);
	}
	
	public double distanceTo(Point point) {
		return Math.sqrt(Math.pow(point.x - this.x, 2) + Math.pow(point.y - this.y, 2));
	}
	
	public Point offset(double xOffset, double yOffset) {
		double newX = x + xOffset; 
		double newY = y + yOffset; 
		return new Point(newX, newY); 
	}

	public Point avgPoint(Point point) {
		double newX = (this.x + point.x) / 2; 
		double newY = (this.y + point.y) / 2;
		return new Point(newX, newY);  
	}
	
	public Waypoint toWaypoint() {
		return new Waypoint(x, y); 
	}
	
	public double slope(Point otherPoint) {
		double xChange = x - otherPoint.x;
		double yChange = y - otherPoint.y; 
		return yChange / xChange; 
	}
}