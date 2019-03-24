package Generation.Splines;

import Generation.Waypoint;

public class CubicBezier extends Spline {

	/*
	 * Requires at least four points P1, C1, C2, P2 in that order
	 * P1 is the first point on the curve
	 * C1 is the first control point and lies on the line tangent to P1 
	 * C2 is the second control point and lies on the line tangent to P2
	 * P2 is the last point on the curve
	 * Any more points will be ignored!
	 */
	//More information about the math can be found here: https://pomax.github.io/bezierinfo/
    @Override
    public Waypoint getWaypoint(double alpha, Waypoint... waypoints) {
    	if(waypoints.length < 4) { 
    		return null; 
    	}
        double x = waypoints[0].x * Math.pow(1 - alpha, 3) + waypoints[1].x * 3 * Math.pow(1 - alpha, 2) * alpha + 
					waypoints[2].x * 3 * (1 - alpha) * Math.pow(alpha, 2) + waypoints[3].x * Math.pow(alpha, 3);
		double y = waypoints[0].y * Math.pow(1 - alpha, 3) + waypoints[1].y * 3 * Math.pow(1 - alpha, 2) * alpha + 
					waypoints[2].y * 3 * (1 - alpha) * Math.pow(alpha, 2) + waypoints[3].y * Math.pow(alpha, 3);
		return new Waypoint(x, y); 
    }

    @Override
    public Waypoint[][] getCurvepoints(double tightness, Waypoint... waypoints) {
    	if(waypoints.length < 2) {
    		System.out.println("Not enough points");
    		return null;
    	}
    	//taken from team 3641's method of calculating bezier curves found here: 
    	//https://github.com/JackToaster/FlyingToasters2018/blob/master/src/path_generation/Path.java
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