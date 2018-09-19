
public class Waypoint extends Point {
	//Universal units so longs as you are consistent
	double time;
	double distanceFromStart; 
	double distanceFromEnd; 
	double velocity; 
	double acceleration; 
	double jerk;  
	double heading; //Counterclockwise and in radians
	double angularVelocity;  
	
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
	
	//Gets the heading of the waypoint in a path counterclockwise and in radians
	/*
	 * @param another waypoint persumably the next one
	 */
	public void getHeading(Waypoint waypoint) {
		double xDiff = waypoint.x - this.x; 
		double yDiff = waypoint.y - this.y;
		
		if(waypoint.x >= this.x - 0.001 && waypoint.x <= this.x + 0.001) {
			if(yDiff > 0) {
				heading = Math.PI / 2;
			}else if(yDiff < 0) {
				heading = 3 * Math.PI / 2; 
			}
		}else if(waypoint.y >= this.y - 0.001 && waypoint.y <= this.y + 0.001) {
			if(xDiff > 0) {
				heading = 0; 
			}else if(xDiff < 0) {
				heading = Math.PI; 
			}
		}else {	
			double opposite = Math.abs(waypoint.y - this.y);
			double adjacent = Math.abs(waypoint.x - this.x);
			
			if(xDiff > 0 && yDiff > 0) {
				heading = Math.atan(opposite / adjacent);
			}else if(xDiff < 0 && yDiff < 0) {
				heading = Math.PI + Math.atan(opposite / adjacent);
			}else if(xDiff < 0 && yDiff > 0) {
				heading = Math.PI - Math.atan(opposite / adjacent);
			}else if(xDiff > 0 && yDiff < 0) {
				heading = 2 * Math.PI - Math.atan(opposite / adjacent);
			}
		}
	}

	//A method to get a perpendicauliar offset of a point
	/*
	 * @param the offset left is positive, right is negative
	 */
	public Waypoint offsetWaypointPerpen(double offset) {
		double angle;
		if(isFowards()) {
			angle = heading + Math.PI / 2;  
		}else {
			angle = heading - Math.PI / 2;
			offset *= -1; 
		}
		Point point = Point.PolarPoint(offset, angle); 
		point = point.offset(x, y); 
		return new Waypoint(point.x, point.y, heading);  
	} 

	//Some methods to determine the direction in which the point is going
	private boolean isFowards() {
        if((heading >= 0 && heading <= Math.PI / 2) || (heading > (3 * Math.PI) / 2 && heading <= 2 * Math.PI)) {
            return true; 
        }else {
            return false; 
        }
    }

    private boolean isBackwards() {
        if(heading > Math.PI / 2 && heading <= (3 * Math.PI) / 2) {
            return true; 
        }else {
            return false; 
        }
    }
}
