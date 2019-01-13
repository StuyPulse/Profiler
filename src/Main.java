
import java.io.File;
import java.io.IOException;
import java.util.Scanner; 

public class Main {
	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in); 
		System.out.println("First lets do some file stuff"); 
		System.out.print("File Destination: "); String fileLocation = scanner.nextLine();
		System.out.print("Trajectory Name: "); String trajName = scanner.nextLine();
		String mode = "Pulse"; 
		System.out.print("Export Mode[Pulse/Jaci]: "); mode = scanner.nextLine();
		
		System.out.println("Next enter some information"); 
		System.out.print("dt: "); double dt = scanner.nextDouble(); 
		System.out.print("Wheel Base Width: "); double wheelBase = scanner.nextDouble(); 
		System.out.print("Max Velocity: "); double maxVelocity = scanner.nextDouble();
		System.out.print("Max Acceleration: "); double maxAcceleration = scanner.nextDouble(); 
		System.out.print("Max Jerk: "); double maxJerk = scanner.nextDouble();
		System.out.print("Velocity Correction?[true/false]: "); boolean velCor = scanner.nextBoolean();  

		System.out.print("Then the waypoints\n"
							+ "Number of waypoints: ");
		int numOfWaypoints = scanner.nextInt(); 
		System.out.println("Add waypoints like so: x(units) y(units) theta(degrees)");
		Waypoint[] waypoints = new Waypoint[numOfWaypoints];
		for(int i = 0; i < waypoints.length; i++) {
			waypoints[i] = new Waypoint(scanner.nextDouble(), scanner.nextDouble(), d2r(scanner.nextDouble())); 
		}   

		Path path = new Path(10000, dt, wheelBase, maxVelocity, maxAcceleration, maxJerk, velCor, waypoints);
		File paths = new File(fileLocation + "\\" + trajName);
		paths.mkdir();
		String central = paths.getPath() + "\\" + trajName + "_central_" + mode + ".csv";
		String left = paths.getPath() + "\\" + trajName + "_left_" + mode + ".csv";
		String right = paths.getPath() + "\\" + trajName + "_right_" + mode + ".csv";
		try {
			switch(mode) {
				case "Jaci": 
					Csv.writeTrajFileJaci(path.getCentralTrajectory(), central, dt);
					Csv.writeTrajFileJaci(path.getLeftTrajectory(), left, dt);
					Csv.writeTrajFileJaci(path.getRightTrajectory(), right, dt);
					break;
				default: 
					Csv.writeTrajFilePulse(path.getCentralTrajectory(), central);
					Csv.writeTrajFilePulse(path.getLeftTrajectory(), left);
					Csv.writeTrajFilePulse(path.getRightTrajectory(), right);
					break;
			}
		}catch (IOException i) {
			System.out.println("Invalid file"); 
			i.printStackTrace();
		}
		scanner.close();
	}
	
	public static double d2r(double degrees) {
		return (degrees * Math.PI) / 180; 
	}
	
	public static double r2d(double radians) {
		return (radians * 180) / Math.PI; 
	}	
}

