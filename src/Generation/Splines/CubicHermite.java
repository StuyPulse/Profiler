package Generation.Splines;

import Generation.Waypoint;

public class CubicHermite extends Spline {

    //Accepts four points P1, P2, T1, T2
    //P1 is first point
    //P2 is last point
    //T1 is a vector containing derivatives at P1
    //T2 is a vector containing derivatives at P2
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
    public Waypoint[][] getCurvepoints(double tightness, Waypoint... waypoints) {
        Waypoint[][] curvepoints = new Waypoint[waypoints.length - 1][4];

        double t1x = waypoints[1].x - waypoints[0].x;
        double t2x = 0.5 * (waypoints[2].x - waypoints[0].x);
        curvepoints[0][0] = waypoints[0];
        curvepoints[0][1] = waypoints[1];
        curvepoints[0][2] = new Waypoint(t1x, t1x * Math.tan(waypoints[0].heading));
        curvepoints[0][3] = new Waypoint(t2x, t2x * Math.tan(waypoints[1].heading));

        for(int i = 1; i < curvepoints.length - 1; i++) {
            curvepoints[i][0] = waypoints[i];
            curvepoints[i][1] = waypoints[i+1];
            t1x = 0.5 * (waypoints[i+1].x - waypoints[i-1].x);
            curvepoints[i][2] = new Waypoint(t1x, t1x * Math.tan(waypoints[i].heading));
            t2x = 0.5 * (waypoints[i+2].x - waypoints[i].x);
            curvepoints[i][3] = new Waypoint(t2x, t2x * Math.tan(waypoints[i].heading));
        }

        int len = waypoints.length;
        t1x = 0.5 * (waypoints[len - 1].x - waypoints[len - 3].x);
        t2x = waypoints[len - 1].x - waypoints[len - 2].x;
        curvepoints[len - 1][0] = waypoints[len - 2];
        curvepoints[len - 1][1] = waypoints[len - 1];
        curvepoints[len - 1][2] = new Waypoint(t1x, t1x * Math.tan(waypoints[len - 2].heading));
        curvepoints[len - 1][3] = new Waypoint(t2x, t2x * Math.tan(waypoints[len - 1].heading));
        return curvepoints;
    }

}