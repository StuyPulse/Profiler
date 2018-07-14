 
public class BezGen {
	public static void main(String[] args) {
		Waypoint[] waypoints = {
				new Waypoint(5, 3, 0), 
				new Waypoint(10, 6, d2r(45)), 
				new Waypoint(15, 9, 0)
		}; 
		Path path = new Path(100, 4, 3, waypoints);
		System.out.println("x  " + "y  " + "distance  " + "velocity  " + "acceleration  " + "heading");
		for(int i = 0; i < path.trajectory.size(); i++) {
			double x = path.trajectory.get(i).x; 
			double y = path.trajectory.get(i).y;
			double distance = path.trajectory.get(i).distanceFromStart;
			double heading = r2d(path.trajectory.get(i).heading);
			double velocity = path.trajectory.get(i).velocity; 
			double acceleration = path.trajectory.get(i).acceleration;
			System.out.println(x + "  " + y + "  " + distance + "  " + velocity + "  " + acceleration + "  " + heading);
		}
	}
	
	public static double d2r(double degrees) {
		return (degrees * Math.PI) / 180; 
	}
	
	public static double r2d(double radians) {
		return (radians * 180) / Math.PI; 
	}
}
