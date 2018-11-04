
import java.io.IOException;
import java.util.Arrays;
import java.util.Scanner; 

public class Main {
	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in); 
		System.out.println("First lets do some file stuff"); 
		System.out.print("File Destination: "); String fileLocation = scanner.nextLine();
		System.out.print("Trajectory Name: "); String trajName = scanner.nextLine();

		System.out.println("Next enter some information"); 
		System.out.print("dt: "); double dt = scanner.nextDouble(); 
		System.out.print("Wheel Base Width: "); double wheelBase = scanner.nextDouble(); 
		System.out.print("Max Velocity: "); double maxVelocity = scanner.nextDouble();
		System.out.print("Max Acceleration: "); double maxAcceleration = scanner.nextDouble(); 
		System.out.print("Max Jerk: "); double maxJerk = scanner.nextDouble();
		System.out.print("Velocity Correction?[true/false] "); 
		boolean velCorrection = scanner.nextLine().toLowerCase().charAt(0) == 't'; 

		System.out.print("Then the waypoints\n"
							+ "Number of waypoints: ");
		int numOfWaypoints = scanner.nextInt(); 
		System.out.println("Add waypoints like so: x(units) y(units) theta(degrees)");
		Waypoint[] waypoints = new Waypoint[numOfWaypoints];
		for(int i = 0; i < waypoints.length; i++) {
			waypoints[i] = new Waypoint(scanner.nextDouble(), scanner.nextDouble(), d2r(scanner.nextDouble())); 
		}   
		
		String mode = ""; 
		while(!(Arrays.asList(Csv.exportModes).contains(mode))) {
			System.out.print("Finally the export mode[Pulse/Jaci]"); mode = scanner.nextLine().toLowerCase(); 
		}

		Path path = new Path(10000, dt, wheelBase, maxVelocity, maxAcceleration, maxJerk, velCorrection, waypoints);
		String central = fileLocation + "\\" + trajName + "_central_" + mode + ".csv";
		String left = fileLocation + "\\" + trajName + "_left_" + mode + ".csv";
		String right = fileLocation + "\\" + trajName + "_right_" + mode + ".csv";
		try {
			switch(mode) {
				case "Pulse":
					Csv.writeTrajFilePulse(path.centralTrajectory, central);
					Csv.writeTrajFilePulse(path.leftTrajectory, left);
					Csv.writeTrajFilePulse(path.rightTrajectory, right);
					break; 
				case "Jaci": 
					Csv.writeTrajFileJaci(path.centralTrajectory, central, dt);
					Csv.writeTrajFileJaci(path.leftTrajectory, left, dt);
					Csv.writeTrajFileJaci(path.rightTrajectory, right, dt);
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

