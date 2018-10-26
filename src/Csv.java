 
import java.io.BufferedReader; 
import java.io.BufferedWriter;
import java.io.FileReader; 
import java.io.FileWriter;
import java.io.IOException; 

import java.util.ArrayList;

public class Csv {
    
    public static BufferedWriter getWriter(String fileStr) throws IOException {
        FileWriter fwriter = new FileWriter(fileStr);
        return new BufferedWriter(fwriter);  
    }
	
	public static void writeLine(BufferedWriter bWriter, String str) throws IOException {
		bWriter.write(str);
		bWriter.newLine();
		bWriter.flush(); 
	}
    
    public static Waypoint readLine(String str) {
        String values[] = str.split(",");
        Waypoint waypoint = new Waypoint(Double.parseDouble(values[1]), Double.parseDouble(values[2]), Double.parseDouble(values[7])); 
        waypoint.time = Double.parseDouble(values[0]); 
        waypoint.distanceFromStart = Double.parseDouble(values[3]); 
        waypoint.velocity = Double.parseDouble(values[4]); 
        waypoint.acceleration = Double.parseDouble(values[5]); 
        waypoint.jerk = Double.parseDouble(values[6]); 
        return waypoint; 
    }

    public static void writeTrajFile(ArrayList<Waypoint> trajectory, String fileStr) throws IOException {
        BufferedWriter bWriter = getWriter(fileStr); 
        writeLine(bWriter, "time,x,y,distance,velocity,acceleration,jerk,heading");
		for(int i = 0; i < trajectory.size(); i++) {
			double time = trajectory.get(i).time;  
			double x = trajectory.get(i).x; 
			double y = trajectory.get(i).y;
			double distance = trajectory.get(i).distanceFromStart;
			double heading = trajectory.get(i).heading;
			double velocity = trajectory.get(i).velocity; 
			double acceleration = trajectory.get(i).acceleration;
			double jerk = trajectory.get(i).jerk; 
            writeLine(bWriter, 
                time + "," + x + "," + y + "," + distance + "," + velocity + "," + acceleration + "," + jerk + "," + heading);
        } 
        bWriter.close(); 
    }

    public static ArrayList<Waypoint> readTrajFile(String fileStr) throws IOException {
        ArrayList<Waypoint> trajectory = new ArrayList<Waypoint>(); 
        FileReader fReader = new FileReader(fileStr); 
        BufferedReader bReader = new BufferedReader(fReader);
        bReader.readLine(); 
        String next = bReader.readLine(); 
        while(next != null) {
            trajectory.add(readLine(next)); 
            next = bReader.readLine(); 
        } 
        bReader.close();
        return trajectory; 
    }
}
