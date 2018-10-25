
import java.io.IOException;

import java.util.ArrayList;
import java.util.Scanner; 

public class Main {
	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in);
		System.out.println("W to write a new file, R to read from a file");
		char function = scanner.nextLine().charAt(0);
		if(function == 'W') { 
			System.out.println("First lets do some file stuff"); 
			System.out.print("File Destination: "); String fileLocation = scanner.nextLine();
			System.out.print("Trajectory Name: "); String trajName = scanner.nextLine();

			System.out.println("Next enter some information"); 
			System.out.print("dt: "); double dt = scanner.nextDouble(); 
			System.out.print("Wheel Base Width: "); double wheelBase = scanner.nextDouble(); 
			System.out.print("Max Velocity: "); double maxVelocity = scanner.nextDouble();
			System.out.print("Max Acceleration: "); double maxAcceleration = scanner.nextDouble(); 
			System.out.print("Max Jerk: "); double maxJerk = scanner.nextDouble();
			System.out.print("Velocity Correction?[true/false] "); boolean velCorrection = scanner.nextBoolean(); 

			System.out.print("Finally the waypoints\n"
								+ "Number of waypoints: ");
			int numOfWaypoints = scanner.nextInt(); 
			System.out.println("Add waypoints like so: x(units) y(units) theta(degrees)");
			Waypoint[] waypoints = new Waypoint[numOfWaypoints];
			for(int i = 0; i < waypoints.length; i++) {
				waypoints[i] = new Waypoint(scanner.nextDouble(), scanner.nextDouble(), d2r(scanner.nextDouble())); 
			}   

			Path path = new Path(10000, dt, wheelBase, maxVelocity, maxAcceleration, maxJerk, velCorrection, waypoints);
			String central = fileLocation + "\\" + trajName + "_central_Pulse.csv";
			String left = fileLocation + "\\" + trajName + "_left_Pulse.csv";
			String right = fileLocation + "\\" + trajName + "_right_Pulse.csv";
			try {
				csv.writeTrajFile(path.centralTrajectory, central);
				csv.writeTrajFile(path.leftTrajectory, left);
				csv.writeTrajFile(path.rightTrajectory, right);
			}catch (IOException i) {
				System.out.println("Invalid file"); 
			}
		}else if(function == 'R') {
			 System.out.print("File Location: "); 
			 try {
				ArrayList<Waypoint> trajectory = csv.readTrajFile(scanner.nextLine());
				//Do whatever you want with the trajectory
			 }catch (IOException i) {
				System.out.println("Invalid file"); 
			 } 
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

