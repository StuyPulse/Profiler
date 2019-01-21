package Application;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import Files.Csv;
import Generation.CenterTraj;
import Generation.SideTraj;
import Generation.Waypoint; 

public class Main {
	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in); 
		System.out.println("First lets do some file stuff"); 
		System.out.print("File Destination: "); String fileLocation = scanner.nextLine();
		System.out.print("Trajectory Name: "); String trajName = scanner.nextLine();
		//System.out.print("Export Mode[Pulse/Jaci]: "); mode = scanner.nextLine();
		
		System.out.println("Next enter some information"); 
		System.out.print("dt: "); double dt = scanner.nextDouble(); 
		System.out.print("Wheel Base Width: "); double wheelBase = scanner.nextDouble(); 
		System.out.print("Max Velocity: "); double maxVelocity = scanner.nextDouble();
		System.out.print("Max Acceleration: "); double maxAcceleration = scanner.nextDouble(); 
		System.out.print("Max Jerk: "); double maxJerk = scanner.nextDouble();
		//System.out.print("Velocity Correction?[true/false]: "); boolean velCor = scanner.nextBoolean();  

		System.out.print("Then the waypoints\n"
							+ "Number of waypoints: ");
		int numOfWaypoints = scanner.nextInt(); 
		System.out.println("Add waypoints like so: x(units) y(units) theta(degrees)");
		Waypoint[] waypoints = new Waypoint[numOfWaypoints];
		for(int i = 0; i < waypoints.length; i++) {
			waypoints[i] = new Waypoint(scanner.nextDouble(), scanner.nextDouble(), d2r(scanner.nextDouble())); 
		}   

		double startTime = getSec();
		System.out.println("Generating");
		CenterTraj centerTraj = new CenterTraj(100000, dt, wheelBase, maxVelocity, maxAcceleration, maxJerk, waypoints);
		centerTraj.generate();
		SideTraj leftTraj = centerTraj.getLeft(); leftTraj.generate();
		SideTraj rightTraj = centerTraj.getRight(); rightTraj.generate();
		System.out.println("Finished! " + (getSec() - startTime) + " sec");
		System.out.println("Exporting");
		String seperator = System.getProperty("os.name").contains("Windows") ? "\\" : "/";
		File trajs = new File(fileLocation + seperator + trajName);
		trajs.mkdir();
		String centralFile = trajs.getPath() + seperator + trajName + "_central_Pulse.csv";
		String leftFile = trajs.getPath() + seperator + trajName + "_left_Pulse.csv";
		String rightFile = trajs.getPath() + seperator + trajName + "_right_Pulse.csv";
		try {
			Csv.writeTrajFilePulse(centerTraj.getPath(), centralFile);
			Csv.writeTrajFilePulse(leftTraj.getPath(), leftFile);
			Csv.writeTrajFilePulse(rightTraj.getPath(), rightFile);
		}catch (IOException i) {
			System.out.println("Invalid file"); 
			i.printStackTrace();
		}
		System.out.println("Finished! " + (getSec() - startTime) + " sec");
		scanner.close();
	}
	
	public static double d2r(double degrees) {
		return (degrees * Math.PI) / 180; 
	}
	
	public static double r2d(double radians) {
		return (radians * 180) / Math.PI; 
	}	
	
	public static long getSec() {
		return (long) System.currentTimeMillis() / (long) 1000;
	}
}

