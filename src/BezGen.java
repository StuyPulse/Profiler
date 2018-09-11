 
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import java.util.Scanner;
import java.util.ArrayList; 

public class BezGen {
	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in);
		System.out.println("First lets do some file stuff"); 
		System.out.print("File Destination: "); String fileLocation = scanner.nextLine();
		System.out.print("Trajectory Name: "); String trajName = scanner.nextLine();

		System.out.println("First enter some information"); 
		System.out.print("dt: "); double dt = scanner.nextDouble(); 
		System.out.print("Wheel Base Width: "); double wheelBase = scanner.nextDouble(); 
		System.out.print("Max Velocity: "); double maxVelocity = scanner.nextDouble();
		System.out.print("Max Acceleration: "); double maxAcceleration = scanner.nextDouble(); 
		System.out.print("Max Jerk (hint-60ft/sec^3): "); double maxJerk = scanner.nextDouble();

		System.out.print("Next the waypoints\n"
							+ "Number of waypoints: ");
		int numOfWaypoints = scanner.nextInt(); 
		System.out.println("Add waypoints like so: x(units) y(units) theta(degrees)");
		Waypoint[] waypoints = new Waypoint[numOfWaypoints];
		for(int i = 0; i < waypoints.length; i++) {
			waypoints[i] = new Waypoint(scanner.nextDouble(), scanner.nextDouble(), d2r(scanner.nextDouble())); 
		}   
		scanner.close();

		Path path = new Path(100, dt, wheelBase, maxVelocity, maxAcceleration, maxJerk, waypoints);
		String central = fileLocation + "\\" + trajName + "_central.csv";
		String left = fileLocation + "\\" + trajName + "_left.csv";
		String right = fileLocation + "\\" + trajName + "_right.csv";
		writeTrajFile(path.centralTrajectory, central);
		writeTrajFile(path.leftTrajectory, left);
		writeTrajFile(path.rightTrajectory, right);
	}
	
	public static double d2r(double degrees) {
		return (degrees * Math.PI) / 180; 
	}
	
	public static double r2d(double radians) {
		return (radians * 180) / Math.PI; 
	}

	public static void writeTrajFile(ArrayList<Waypoint> trajectory, String fileStr) {
		Writer writer = new Writer(fileStr);
		writer.writeLine("time,x,y,distance,velocity,acceleration,jerk,heading");
		for(int i = 0; i < trajectory.size(); i++) {
			double time = trajectory.get(i).time; 
			double x = trajectory.get(i).x; 
			double y = trajectory.get(i).y;
			double distance = trajectory.get(i).distanceFromStart;
			//Headings should be exported in radians, just exporting in degrees for tests
			double heading = r2d(trajectory.get(i).heading);
			double velocity = trajectory.get(i).velocity; 
			double acceleration = trajectory.get(i).acceleration;
			double jerk = trajectory.get(i).jerk; 
			writer.writeLine(time + "," + x + "," + y + "," + distance + "," + velocity + "," + acceleration + "," + jerk + "," + heading);
		} 
	}
}

class Writer {
	File file; 
	FileWriter fWriter; 
	BufferedWriter bWriter;
	boolean closed; 

	public Writer(String fileStr) {
		try {
			file = new File(fileStr); 
			fWriter = new FileWriter(file); 
			bWriter = new BufferedWriter(fWriter);
			closed = false; 
		}catch(Exception e) {
			System.out.println("Error");
		} 
	}
	
	public void writeLine(String str) {
		try {
			bWriter.write(str);
			bWriter.newLine();
			bWriter.flush(); 
		}catch(Exception e) {
			System.out.println("Error"); 
		}
	}

	public void closeFile() {
		if(!closed) {
			try {
				bWriter.close();
				closed = true;
			}catch(Exception e) {
				System.out.println("Error");
			}
		}else {
			System.out.println("File already closed");
		}
	}
}
