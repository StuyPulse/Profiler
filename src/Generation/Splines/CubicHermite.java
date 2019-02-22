package Generation.Splines;

import Generation.Waypoint;

public class CubicHermite extends Spline {

    /*
     * Requires at least four points P1, P2, T1, T2 in that order
     * P1 is the first point on the curve
     * P2 is the last point on the curve
     * T1 is a vector with xy derivatives at P1
     * T2 is a vector with xy derivatives at P2
     * Any more points will be ignored!
     */
	//More information about the math can be found here: http://www.cs.cornell.edu/courses/cs4620/2012fa/lectures/28splines.pdf
	//And here: https://www.cubic.org/docs/hermite.htm
    @Override
    public Waypoint getWaypoint(double alpha, Waypoint... waypoints) {
        double h1 = 2 * Math.pow(alpha, 3) - 3 * Math.pow(alpha, 2) + 1; 
        double h2 = -2 * Math.pow(alpha, 3) + 3 * Math.pow(alpha, 2);
        double h3 = Math.pow(alpha, 3) - 2 * Math.pow(alpha, 2) + alpha; 
        double h4 = Math.pow(alpha, 3) - Math.pow(alpha, 2);
        double x = waypoints[0].x * h1 + waypoints[1].x * h2 + 
                    waypoints[2].x * h3 + waypoints[3].x * h4; 
        double y = waypoints[0].y * h1 + waypoints[1].y * h2 + 
                    waypoints[2].y * h3 + waypoints[3].y * h4;            
        return new Waypoint(x, y);
    }

    @Override
    public Waypoint[][] getCurvepoints(Waypoint... waypoints) {
    	if(waypoints.length < 2) {
    		System.out.println("Not enough points");
    		return null;
    	}
    	Waypoint[][] curvepoints = new Waypoint[waypoints.length - 1][4];
    	for(int i = 0; i < curvepoints.length; i++) {
    		//taken from 254 cubic spline generation code: 
    		//https://github.com/Team254/FRC-2018-Public/blob/master/cheesy-path/src/main/java/com/team254/lib/spline/CubicHermiteSpline.java
    		double scale = 2 * waypoints[i].distanceTo(waypoints[i+1]);
    		//these are the waypoints themselves
    		curvepoints[i][0] = waypoints[i]; 
    		curvepoints[i][1] = waypoints[i+1]; 
    		//these are more or less vectors containing the derivatives at the two points
    		//atan2(dy, dx) = heading
    		//so, cos(heading) * scale = dx and sin(heading) * scale = dy 
    		curvepoints[i][2] = new Waypoint(Math.cos(waypoints[i].heading) * scale, Math.sin(waypoints[i].heading) * scale);
    		curvepoints[i][3] = new Waypoint(Math.cos(waypoints[i+1].heading) * scale, Math.sin(waypoints[i+1].heading) * scale);
    	}
        return curvepoints;
    }

}