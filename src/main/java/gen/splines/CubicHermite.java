package gen.splines;

import gen.Waypoint;

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
        if(waypoints.length < 4) {
            return null;
        }
        double h1 = 2 * Math.pow(alpha, 3) - 3 * Math.pow(alpha, 2) + 1; 
        double h2 = -2 * Math.pow(alpha, 3) + 3 * Math.pow(alpha, 2);
        double h3 = Math.pow(alpha, 3) - 2 * Math.pow(alpha, 2) + alpha; 
        double h4 = Math.pow(alpha, 3) - Math.pow(alpha, 2);
        double x = waypoints[0].x * h1 + waypoints[1].x * h2 + 
                    waypoints[2].x * h3 + waypoints[3].x * h4; 
        double y = waypoints[0].y * h1 + waypoints[1].y * h2 + 
                    waypoints[2].y * h3 + waypoints[3].y * h4;
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
		//taken from 254 cubic spline generation code: 
		//https://github.com/Team254/FRC-2018-Public/blob/master/cheesy-path/src/main/java/com/team254/lib/spline/CubicHermiteSpline.java
    	Waypoint[][] curvepoints = new Waypoint[waypoints.length - 1][4];
    	for(int i = 0; i < curvepoints.length; i++) {
    		//scaling is just like with bezier curves except that it is x3 explanation can be found below: 
    		//http://www2.cs.uregina.ca/~anima/408/Notes/Interpolation/BezierDerivation.htm
    		double scale = waypoints[i].distanceTo(waypoints[i+1]) / 2 * tightness * 3;
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

    @Override
    public Waypoint differentiate(double alpha, Waypoint... waypoints) {
        if(waypoints.length < 4) {
            return null;
        }
        /* to find the basis functions:
         * sub the values of a,b,c,d in the equation f'(x) = 3at^2 + 2b + c
         * multiply the basis functions by the spline points to find the derivative at alpha
         */
        double h1 = 6 * Math.pow(alpha, 2) - 6 * alpha;
        double h2 = -6 * Math.pow(alpha, 2) + 6 * alpha;
        double h3 = 3 * Math.pow(alpha, 2) - 4 * alpha + 1;
        double h4 = 3 * Math.pow(alpha, 2) - 2 * alpha;
        double dx = waypoints[0].x * h1 + waypoints[1].x * h2 +
                waypoints[2].x * h3 + waypoints[3].x * h4;
        double dy = waypoints[0].y * h1 + waypoints[1].y * h2 +
                waypoints[2].y * h3 + waypoints[3].y * h4;
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
        for(int i = 0; i < 2; i++) {
            double pt = ((to - from) / 2.0) * abscissa[i] + ((to + from) / 2.0);
            Waypoint d = differentiate(pt, waypoints);
            sum += weights[i] * Math.sqrt(d.x*d.x + d.y*d.y);
        }
        return ((to - from) / 2.0) * sum;
    }

}