package gen.segments;

import gen.Vector;
import gen.Waypoint;

/**
 * CubicBezierSegment.java
 *
 * @author Tahsin Ahmed
 *
 * A segment that is generated through the cubic bezier equations.
 */

public class CubicBezierSegment extends Segment {
    /** A cubic bezier segment generator
     * used by the corresponding spline.  */
    public static class CubicBezierSegmentFactory implements Segment.SegmentFactory {
        /**
         * @param tightness A multiplier that controls how close the
         *                  control points are to the start/end of
         *                  the individual curve.
         * @param startwp the starting waypoint.
         * @param endwp the ending waypoint
         * @return A segment that connects the starting waypoint
         * to the ending waypoint witht he given tightness (this
         * will stay constant throughout the generation of the curve).
         */
        @Override
        public CubicBezierSegment getInstance(double tightness, Waypoint startwp, Waypoint endwp) {
            // taken from team 3641's method of calculating bezier curves found here:
            // https://github.com/JackToaster/FlyingToasters2018/blob/master/src/path_generation/Path.java
            double scale = startwp.distanceTo(endwp) * tightness;

            Vector[] points = new Vector[4];
            points[0] = startwp;
            points[3] = endwp;

            points[1] = startwp.offset(Math.cos(startwp.heading) * scale / 3, Math.sin(startwp.heading) * scale / 3).toVector();
            points[2] = endwp.offset(Math.cos(endwp.rotate(Math.PI).heading) * scale / 3, Math.sin(endwp.rotate(Math.PI).heading) * scale / 3).toVector();

            return new CubicBezierSegment(points);
        }

    }

    /**
     * 3 is the order of the segment.
     * @param points the control and starting points.
     */
    public CubicBezierSegment(Vector... points) {
        super(3, points);
    }

    /**
     * @param alpha spline parameter [0, 1]
     *              indicates progression on curve.
     * @return the point on a curve (in vector form).
     */
    @Override
    public Vector getCors(double alpha) {
        double h0 = Math.pow(1 - alpha, 3);
        double h1 = 3 * Math.pow(1 - alpha, 2) * alpha;
        double h2 = 3 * (1 - alpha) * Math.pow(alpha, 2);
        double h3 = Math.pow(alpha, 3);
        double x = points[0].x * h0 + points[1].x * h1 +
                points[2].x * h2 + points[3].x * h3;
        double y = points[0].y * h0 + points[1].y * h1 +
                points[2].y * h2 + points[3].y * h3;
        Vector point = new Vector(x, y);
        return point;
    }

    /**
     * @param alpha spline parameter [0, 1]
     *              indicates progression on curve.
     * @return a Vector that contains the x
     *         and y components of the derivative at that point.
     */
    @Override
    public Vector differentiate(double alpha) {
        // uses equation for bezier derivatives: Σi=0,n Bn-1,i(t) * n * (Pi+1 - pi)
        // https://pomax.github.io/bezierinfo/#derivatives
        double dh0 = Math.pow(1 - alpha, 2);
        double dh1 = 2 * (1- alpha) * alpha;
        double dh2 = Math.pow(alpha, 2);
        double dx = 3 * (points[1].x - points[0].x) * dh0 +
                3 * (points[2].x - points[1].x) * dh1 +
                3 * (points[3].x - points[2].x) * dh2;
        double dy = 3 * (points[1].y - points[0].y) * dh0 +
                3 * (points[2].y - points[1].y) * dh1 +
                3 * (points[3].y - points[2].y) * dh2;
        return new Vector(dx, dy);
    }

    /**
     * @param alpha spline parameter from [0, 1]
     *              indicates progression on curve.
     * @return second derivative (slope of slope)
     *              as (x, y) vector.
     */
    @Override
    public Vector differentiateS(double alpha) {
        double ddh0 = 1 - alpha;
        double ddh1 = alpha;
        double ddx = 6 * (points[2].x - 2*points[1].x + points[0].x) * ddh0 +
                    6 * (points[3].x - 2*points[2].x + points[1].x) * ddh1;
        double ddy = 6 * (points[2].y - 2*points[1].y + points[0].y) * ddh0 +
                6 * (points[3].y - 2*points[2].y + points[1].y) * ddh1;
        return new Vector(ddx, ddy);
    }

    /**
     * @param from a point on the curve.
     * @param to another point on the curve.
     * @return the arc length between these two points.
     */
    @Override
    public double integrate(double from, double to) {
        // uses 2 point Gauss Quadrature method to approximate arc length: ∫a,b f(x)dx = Σi=0,n Ci*f(xi)
        // https://pomax.github.io/bezierinfo/#arclength
        // https://www.youtube.com/watch?v=unWguclP-Ds&feature=BFa&list=PLC8FC40C714F5E60F&index=1
        // values : https://pomax.github.io/bezierinfo/legendre-gauss.html
        final double[] WEIGHTS = {1.0000000000000000, 1.0000000000000000};
        final double[] ABSCISSA = {-0.5773502691896257, 0.5773502691896257};
        double sum = 0;
        for(int i = 0; i < 2; i++) {
            double pt = ((to - from) / 2.0) * ABSCISSA[i] + ((to + from) / 2.0);
            Vector d = differentiate(pt);
            sum += WEIGHTS[i] * Math.sqrt(d.x*d.x + d.y*d.y);
        }
        return ((to - from) / 2.0) * sum;
    }

}
