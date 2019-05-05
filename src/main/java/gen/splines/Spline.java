package gen.splines;

import gen.Waypoint;

import java.util.ArrayList;

public abstract class Spline {
    public abstract Waypoint getWaypoint(double alpha, Waypoint... waypoints);
    
    public abstract Waypoint[][] getSplines(double tightness, Waypoint... waypoints);

    public ArrayList<Waypoint> getPath(double sampleRate, Waypoint[][] curvepoints) {
        ArrayList<Waypoint> traj = new ArrayList<Waypoint>();
        traj.add(getWaypoint(0, curvepoints[0]));
        for(Waypoint[] spline : curvepoints) {
            for(int j = 1; j <= sampleRate; j++) {
				double percentage =  j / sampleRate;
				traj.add(getWaypoint(percentage, spline));
            }    
        }
        return traj;
    }

    /*
     * @param spline parameter ∈ [0,1]
     * @param waypoints of spline
     * @return dx, dy of point at alpha on waypoints spline
     */
    public abstract Waypoint differentiate(double alpha, Waypoint... waypoints);

    /*
     * @param spline parameter ∈ [0,1], starting point
     * @param spline parameter ∈ [0,1], ending point
     * @param waypoints of spline
     * @return dx, dy of point at alpha on waypoints spline
     */
    public abstract double integrate(double from, double to, Waypoint... waypoints);
}