 
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import java.util.Scanner;
import java.util.ArrayList; 

public class BezGen {
	public static void main(String[] args) {
		//Insert waypoints here
		//use the method d2r() if you are giving headings in degrees
		Waypoint[] waypoints = {
				//Examples

				//Going fowards
				/*new Waypoint(5, 3, 0), 
				new Waypoint(10, 6, d2r(45)), 
				new Waypoint(15, 9, 0)*/

				//Going Backwards
				/*new Waypoint(15, 9, d2r(180)),
				new Waypoint(10, 6, d2r(225)), 
				new Waypoint(5, 3, d2r(180))*/

				//Changing orientation in the path
				//DOSEN'T WORK!
				/*new Waypoint(5, 5, 0), 
				new Waypoint(10, 10, d2r(90)), 
				new Waypoint(5, 15, d2r(180))*/
		}; 
		//In the constructor define properties of the curve 
		//sample rate, wheel base width, max velocity, max acceleration, max jerk, and the waypoints that were defined in the previous array
		//look at the class for more details
		Path path = new Path(100, 1.936, 4.0, 3.0, 60.0, waypoints);
		
		Scanner scanner = new Scanner(System.in); 
		System.out.println("File Destination: ");
		String fileLocation = scanner.nextLine();
		System.out.println("Trajectory Name: ");
		String trajName = scanner.nextLine();
		scanner.close();
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
