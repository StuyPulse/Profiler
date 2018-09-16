
public class Point {
	double x; 
	double y; 
	
	//Regular point
	//@param x coordinate
	//@param y coordinate
	public Point(double x, double y) {
		this.x = x; 
		this.y = y; 
	}
	
	//Polar point
	//@param distance form the origin 
	//@param rotation counterclockwise from the positive x axis in radians 
	public static Point PolarPoint(double distance, double rotation) {
		double x = Math.cos(rotation) * distance;
		double y = Math.sin(rotation) * distance;
		return new Point(x, y);
	}
	
	double distanceTo(Point point) {
		return Math.sqrt(Math.pow(point.x - this.x, 2) + Math.pow(point.y - this.y, 2));
	}
	
	Point offset(double xOffset, double yOffset) {
		double newX = x + xOffset; 
		double newY = y + yOffset; 
		return new Point(newX, newY); 
	}

	Point avgPoint(Point point) {
		double newX = (this.x + point.x) / 2; 
		double newY = (this.y + point.y) / 2;
		return new Point(newX, newY);  
	}
	
	Waypoint toWaypoint() {
		return new Waypoint(x, y); 
	}
	
	//A lot of basic math stuff
	
	double slope(Point otherPoint) {
		double xChange = x - otherPoint.x;
		double yChange = y - otherPoint.y; 
		return yChange / xChange; 
	}
	
	double perpenSlope(Point otherPoint) {
		double xChange = x - otherPoint.x;
		//Dealing with cases of indefinied slope
		if (xChange < 0.5 && xChange > -0.5) {
			return 0; 
		} 
		return -1 / slope(otherPoint); 
	}
	
	double linear(double slope, double x) {
		return slope * (x - this.x) + y; 
	}
	
	double linear(Point otherPoint, double x) {
		return linear(slope(otherPoint), x); 
	}
	
	double perpen(double slope, double x) {
		return linear(-1 / slope, x);  
	}
	
	double perpen(Point otherPoint, double x) {
		return linear(perpenSlope(otherPoint), x); 
	}

	//More math stuff used for matrix math
	
	double[] linearEquation(double slope) {
		double x = slope; 
		double y = -1; 
		double result = -this.y + slope * this.x; 
		double[] matrix = {x, y, result};
		return matrix;
	}
	  
	double[] linearEquation(Point otherPoint) {
		double slope = slope(otherPoint);  
		return linearEquation(slope);  
	}
	  
	double[] perpenEquation(double slope) {
		double perpenSlope = -1 / slope; 
		return linearEquation(perpenSlope); 
	}
	  
	double[] perpenEquation(Point otherPoint) {
		double perpenSlope = perpenSlope(otherPoint); 
		return linearEquation(perpenSlope);  
	}
	
	//Offsets points in the perpendicular direction
	//By using the elementary approach of guessing and checking numbers
	/******Experimental******/
	/*
	 * @param the point you want to offset
	 * @param another point to compare to which will form a line
	 * @param the offset value
	 * @param the rate at which it will go through the numbers
	 * @param the range at which you want the answer to be in other words the accuracy
	 * @param how the x changes respective of which way you are going
	 * @param how the y changes respective of which way you are going
	 */
	//Warning this could end up not working, but it works at least for most of the things I'd want to do
	Point offsetPerpendicular(Point otherPoint, double offset, double change, double accuracy, Orientation orientation) {
		if(this.x == otherPoint.x && this.y == otherPoint.y) {
			return this;
		}
		double slope = slope(otherPoint);
		if(slope > -1 && slope < 1) {
			if(orientation == Orientation.FOWARDS) {
				return new Point(x, y + -offset); 
			}else if(orientation == Orientation.BACKWARDS) {
				return new Point(x, y + offset);
			}
		}else if(Math.abs(this.x - otherPoint.x) > -1 && Math.abs(this.x - otherPoint.x) < 1) {
			if(orientation == Orientation.FOWARDS) {
				return new Point(x + offset, y); 
			}else {
				return new Point(x - offset, y); 
			}
		}
		if(orientation == Orientation.BACKWARDS) {
			change *= -1; 
		} 
		offset = Math.abs(offset); 
		double lastDist = 0;
		//Iterate through numbers starting at the x value of the point
		for(double i = x; lastDist <= offset; i += change) {
			//Find the corresponding y value
			double perpenY = perpen(otherPoint, i);
			//Find the distance from the starting point
			double dist = distanceTo(new Point(i, perpenY));
			//If it's within an accuracy return it
			if(dist >= offset - accuracy && dist <= offset + accuracy) {
				return new Point(i, perpenY); 
			}
			lastDist = dist; 
		}
		//Return the original point if nothing matches up
		System.out.println("Failure to offset"); 
		return this; 
	}
}
