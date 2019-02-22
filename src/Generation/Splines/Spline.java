package Generation.Splines;

import java.util.ArrayList;

import Generation.Waypoint;

public abstract class Spline {
    public abstract Waypoint getWaypoint(double alpha, Waypoint... waypoints);
    
    public abstract Waypoint[][] getCurvepoints(Waypoint... waypoints);

    public ArrayList<Waypoint> getPath(double sampleRate, Waypoint[][] curvepoints) {
        ArrayList<Waypoint> traj = new ArrayList<Waypoint>();
        traj.add(curvepoints[0][0]); 
        for(Waypoint[] spline : curvepoints) {
            for(int j = 1; j <= sampleRate; j++) {
				double percentage = (double) j / (double) sampleRate;
				Waypoint pathPoint = getWaypoint(percentage, spline); 
                traj.add(pathPoint);
            }    
        }
        return traj;
    };  
}