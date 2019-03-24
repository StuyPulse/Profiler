package Application;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import Files.Csv;
import Files.JSON;
import Files.JSON.JSONArray;
import Files.JSON.JSONObject;
import Generation.CenterTraj;
import Generation.SideTraj;
import Generation.Waypoint;
import Generation.CenterTraj.FitMethod; 

public class Main {
	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in); 
		System.out.println("First lets do some file stuff"); 
		System.out.print("File Destination: "); String fileLocation = scanner.nextLine();
		System.out.print("Trajectory Name: "); String trajName = scanner.nextLine();
		
		System.out.println("Next enter some information");
		System.out.println("[Cubic Bezier, Cubic Hermite]");
		System.out.print("Fit Method: "); String m_str = scanner.nextLine();
		FitMethod method = null; 
		switch(m_str) {
			case "Cubic Bezier": 
				method = FitMethod.CUBIC_BEZIER; 
				break; 
			case "Cubic Hermite": 
				method = FitMethod.CUBIC_HERMITE; 
				break; 
		}
		System.out.print("dt: "); double dt = scanner.nextDouble(); 
		System.out.print("Wheel Base Width: "); double wheelBase = scanner.nextDouble(); 
		System.out.print("Max Velocity: "); double maxVelocity = scanner.nextDouble();
		System.out.print("Max Acceleration: "); double maxAcceleration = scanner.nextDouble(); 
		System.out.print("Max Jerk: "); double maxJerk = scanner.nextDouble();

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
		CenterTraj centerTraj = new CenterTraj(method, 100000, 0.8, dt, wheelBase, maxVelocity, maxAcceleration, maxJerk, waypoints);
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
			Csv.writeTrajFile(centerTraj.getPath(), centralFile);
			Csv.writeTrajFile(leftTraj.getPath(), leftFile);
			Csv.writeTrajFile(rightTraj.getPath(), rightFile);
		}catch (IOException i) {
			System.out.println("Invalid file"); 
			i.printStackTrace();
		}
		System.out.println("Finished! " + (getSec() - startTime) + " sec");
		System.out.print("Save?[Y/n]");
		String save = scanner.nextLine();
		if(scanner.nextLine().toLowerCase().contains("y")) {
			System.out.println("Saving");
			String settingsFile = trajs.getPath() + seperator + trajName + ".json";
			JSONObject data = new JSONObject(); 
		
			data.put("Name", trajName);
			data.put("Fit Method", method.toString());
			data.put("dt", dt);
			data.put("Wheel Base Width", wheelBase);
			JSONArray _waypoints = new JSONArray();
			for(int i = 0; i < waypoints.length; i++) {
				JSONArray waypoint = new JSONArray();
				waypoint.add(waypoints[i].x);
				waypoint.add(waypoints[i].y);
				// Round to the nearest hundredths so there aren't long numbers
				waypoint.add(round(r2d(waypoints[i].heading), 2));
				_waypoints.add(waypoint);
			}
			data.put("Waypoints", _waypoints);
			data.put("Max Velocity", maxVelocity);
			data.put("Max Acceleration", maxAcceleration);
			data.put("Max Jerk", maxJerk);
			
			try {
				JSON.writeJSONFile(data, settingsFile);
			}catch (IOException i) {
				System.out.println("Invalid file"); 
				i.printStackTrace();
			}
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
	
	public static double round(double number, int places) { 
		int current = (int) (number * Math.pow(10, places)) % 10;
		int next = (int) (number * Math.pow(10, places + 1)) % 10;
		double rounded = (int) (number * Math.pow(10, places)) / Math.pow(10, places);
		if(next >= 5) { 
			rounded += Math.pow(10, places * -1);  
		}
		return rounded; 
	}
	
	public static long getSec() {
		return (long) System.currentTimeMillis() / (long) 1000;
	}
}

