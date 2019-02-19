package Generation.Splines;

import Generation.Waypoint;

public class CubicBezier extends Spline {

    @Override
    public Waypoint getWaypoint(double alpha, Waypoint... waypoints) {
        double x = waypoints[0].x * Math.pow(1 - alpha, 3) + waypoints[1].x * 3 * Math.pow(1 - alpha, 2) * alpha + 
					waypoints[2].x * 3 * (1 - alpha) * Math.pow(alpha, 2) + waypoints[3].x * Math.pow(alpha, 3);
		double y = waypoints[0].y * Math.pow(1 - alpha, 3) + waypoints[1].y * 3 * Math.pow(1 - alpha, 2) * alpha + 
					waypoints[2].y * 3 * (1 - alpha) * Math.pow(alpha, 2) + waypoints[3].y * Math.pow(alpha, 3);
		return new Waypoint(x, y); 
    }

    @Override
    public Waypoint[][] getCurvepoints(double tightness, Waypoint... waypoints) {
        Waypoint[][] curvepoints = new Waypoint[waypoints.length - 1][4];
		for(int i = 0; i < waypoints.length - 1; i++) {
			Waypoint startwp = waypoints[i]; 
			Waypoint endwp = waypoints[i + 1];
			double distance = startwp.distanceTo(endwp);
			double gplength = distance / 2 * tightness; 
			Waypoint startOffset = Waypoint.PolarPoint(gplength, startwp.heading);
			Waypoint endOffset = Waypoint.PolarPoint(-gplength, endwp.heading);
			Waypoint control1 = startwp.offset(startOffset.x, startOffset.y);
			Waypoint control2 = endwp.offset(endOffset.x, endOffset.y);
			
			curvepoints[i][0] = startwp; 
			curvepoints[i][1] = control1;
			curvepoints[i][2] = control2;
			curvepoints[i][3] = endwp;  
        }
        return curvepoints; 
    }

}