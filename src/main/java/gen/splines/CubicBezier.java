package gen.splines;

import gen.Waypoint;

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
		Waypoint point = new Waypoint(x, y);
		point.alpha = alpha;
		point.spline = waypoints;
		return point;
    }

    @Override
    public Waypoint[][] getSplines(double tightness, Waypoint... waypoints) {
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
			Waypoint control1 = startwp.offsetCartesian(startOffset.x, startOffset.y);
			Waypoint control2 = endwp.offsetCartesian(endOffset.x, endOffset.y);
			
			curvepoints[i][0] = startwp; 
			curvepoints[i][1] = control1;
			curvepoints[i][2] = control2;
			curvepoints[i][3] = endwp;  
        }
        return curvepoints; 
    }

    @Override
	public Waypoint differentiate(double alpha, Waypoint... waypoints) {
		if(waypoints.length < 4) {
			return null;
		}
		//uses equation for bezier derivatives: Σi=0,n Bn-1,i(t) * n * (Pi+1 - pi)
		//https://pomax.github.io/bezierinfo/#derivatives
		double dx = 3 * Math.pow(1 - alpha, 2) * (waypoints[1].x - waypoints[0].x) +
					6 * alpha * (1 - alpha) * (waypoints[2].x - waypoints[1].x) +
					3 * Math.pow(alpha, 2) * (waypoints[3].x - waypoints[2].x);
		double dy = 3 * Math.pow(1 - alpha, 2) * (waypoints[1].y - waypoints[0].y) +
					6 * alpha * (1 - alpha) * (waypoints[2].y - waypoints[1].y) +
					3 * Math.pow(alpha, 2) * (waypoints[3].y - waypoints[2].y);
    	return new Waypoint(dx, dy);
	}

	@Override
	public double integrate(double from, double to, Waypoint... waypoints) {
		if(waypoints.length < 4) {
			return Double.NaN;
		}
		//uses 2 point Gauss Quadrature method to approximate arc length: ∫a,b f(x)dx = Σi=0,n Ci*f(xi)
		//https://pomax.github.io/bezierinfo/#arclength
		//https://www.youtube.com/watch?v=unWguclP-Ds&feature=BFa&list=PLC8FC40C714F5E60F&index=1
		//values found here: https://pomax.github.io/bezierinfo/legendre-gauss.html
		double[] weights = {1.0000000000000000, 1.0000000000000000};
		double[] abscissa = {-0.5773502691896257, 0.5773502691896257};
		double sum = 0;
		for(int i = 0; i < 3; i++) {
			double pt = ((to - from) / 2.0) * abscissa[i] + ((to + from) / 2.0);
			Waypoint d = differentiate(pt, waypoints);
			sum += weights[i] * Math.sqrt(d.x*d.x + d.y*d.y);
		}
		return ((to - from) / 2.0) * sum;
	}

}