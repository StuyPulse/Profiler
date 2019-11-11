package gen.segments;

import gen.Vector;
import gen.Waypoint;

/**
 * QuinticBezierSegment.java
 *
 * @author Tahsin Ahkmed
 *
 * A segment that is generated through the cubic bezier equations.
 */

public class QuinticBezierSegment extends Segment {
    /** A quintic bezier segment generator that
     * is used by the corresponding spline.
     */

    public static class QuinticBezierSegmentFactory implements Segment.SegmentFactory {
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
        public QuinticBezierSegment getInstance(double tightness, Waypoint startwp, Waypoint endwp) {
            double scale = startwp.distanceTo(endwp) * tightness;

            Vector[] points = new Vector[6];
            points[0] = startwp;
            points[5] = endwp;

            points[1] = startwp.offset(Math.cos(startwp.heading) * scale * 0.2, Math.sin(startwp.heading) * scale * 0.2).toVector();
            points[4] = endwp.offset(Math.cos(-endwp.heading) * scale * 0.2, Math.sin(-endwp.heading) * scale * 0.2).toVector();

            points[2] = startwp.offset( Math.cos(startwp.heading) * scale * 0.4, Math.sin(startwp.heading) * scale * 0.4).toVector();
            points[3] = endwp.offset(Math.cos(-endwp.heading) * scale * 0.4, Math.sin(-endwp.heading) * scale * 0.4).toVector();

            return new QuinticBezierSegment(points);
        }
    }

    /**
     * @param points the control and starting points.
     */
    public QuinticBezierSegment(Vector... points) {
        super(5, points);
    }

    /**
     * @param alpha spline parameter [0, 1]
     *              indicates progression on curve.
     * @return the point on a curve (in vector form).
     */
    @Override
    public Vector getCors(double alpha) {
        double f0 = Math.pow(1 - alpha, 5);
        double f1 = 5 * Math.pow(1 - alpha, 4) * alpha;
        double f2 = 10 * Math.pow(1 - alpha, 3) * Math.pow(alpha, 2);
        double f3 = 10 * Math.pow(1 - alpha, 2) * Math.pow(alpha, 3);
        double f4 = 5 * (1 - alpha) * Math.pow(alpha, 4);
        double f5 = Math.pow(alpha, 5);
        double x = points[0].x * f0 + points[1].x * f1 + points[2].x * f2 + points[3].x * f3 +
                points[4].x * f4 + points[5].x * f5;
        double y = points[0].y * f0 + points[1].y * f1 + points[2].y * f2 + points[3].y * f3 +
                points[4].y * f4 + points[5].y * f5;
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
    public Vector differentiateVector(double alpha) {
        // uses equation for bezier derivatives: Σi=0,n Bn-1,i(t) * n * (Pi+1 - pi)
        // https://pomax.github.io/bezierinfo/#derivatives
        double f0 = Math.pow(1 - alpha, 4);
        double f1 = 4 * alpha * Math.pow(1 - alpha, 3);
        double f2 = 6 * Math.pow(alpha, 2) * Math.pow(1 - alpha, 2);
        double f3 = 4 * Math.pow(alpha, 3) * (1 - alpha);
        double f4 = Math.pow(alpha, 4);
        double dx = 5 * (points[1].x - points[0].x) * f0 + 5 * (points[2].x - points[1].x) * f1 +
                5 * (points[3].x - points[2].x) * f2 + 5 * (points[4].x - points[3].x) * f3 +
                5 * (points[5].x - points[4].x) * f4;
        double dy = 5 * (points[1].y - points[0].y) * f0 + 5 * (points[2].y - points[1].y) * f1 +
                5 * (points[3].y - points[2].y) * f2 + 5 * (points[4].y - points[3].y) * f3 +
                5 * (points[5].y - points[4].y) * f4;
        return new Vector(dx, dy);
    }

    /**
     * @param from a point on the curve.
     * @param to another point on the curve.
     * @return the arc length between these two points.
     */
    @Override
    public double integrate(double from, double to) {
        // uses 3 point Gauss Quadrature method to approximate arc length: ∫a,b f(x)dx = Σi=0,n Ci*f(xi)
        // values found here: https://pomax.github.io/bezierinfo/legendre-gauss.html
        final double[] WEIGHTS = {0.5555555555555556, 0.8888888888888888, 0.5555555555555556};
        final double[] ABSCISSA = {-0.7745966692414834, 0.0000000000000000, 0.7745966692414834};
        double sum = 0;
        for (int i = 0; i < 3; i++) {
            double transformedPt = ((to - from) / 2.0) * ABSCISSA[i] + ((to + from) / 2.0);
            Vector d = differentiateVector(transformedPt);
            sum += WEIGHTS[i] * Math.sqrt(d.x * d.x + d.y * d.y);
        }
        return ((to - from) / 2.0) * sum;
    }

}
