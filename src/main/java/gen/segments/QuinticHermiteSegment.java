package gen.segments;

import gen.Vector;
import gen.Waypoint;

/**
 * QuinticHermiteSegment.java
 * @author Willis Ng
 * A segment that is generated through the quintic hermite equations.
 */
public class QuinticHermiteSegment extends Segment {
    /** A Quintic Hermite segment generator that is
     * used by the corresponding spline.  */
    public static class QuinticHermiteSegmentFactory implements Segment.SegmentFactory {
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
        public QuinticHermiteSegment getInstance(double tightness,
                                               Waypoint startwp, Waypoint endwp) {
            double scale = startwp.distanceTo(endwp) * tightness;

            Vector[] points = new Vector[6];
            points[0] = startwp;
            points[1] = endwp;

            points[2] = new Vector(Math.cos(startwp.heading) * scale, Math.sin(startwp.heading) * scale);
            points[3] = new Vector(Math.cos(endwp.heading) * scale, Math.sin(endwp.heading) * scale);

            points[4] = new Vector(0, 0);
            points[5] = new Vector(0, 0);

            return new QuinticHermiteSegment(points);
        }

    }

    /**
     * 5 is the order of the segment.
     * @param points the control and starting points.
     */
    public QuinticHermiteSegment(Vector... points) {
        super(5, points);
    }

    /**
     * @param alpha spline parameter [0, 1]
     *              indicates progression on curve.
     * @return the point on a curve (in vector form).
     */
    @Override
    public Vector getCors(double alpha) {
        // Quintic Hermite basis functions.
        double p0 = 1 - 10 * Math.pow(alpha, 3) + 15 *
                Math.pow(alpha, 4) - 6 * Math.pow(alpha, 5);
        double p1 = 10 * Math.pow(alpha, 3) - 15 * Math.pow(alpha, 4) +
                6 * Math.pow(alpha, 5);
        double dp0 = alpha - 6 * Math.pow(alpha, 3) +  8 *
                Math.pow(alpha, 4) - 3 * Math.pow(alpha, 5);
        double dp1 = -4 * Math.pow(alpha, 3) + 7 * Math.pow(alpha, 4) - 3 *
                Math.pow(alpha, 5);
        double dpp0 = 0.5 * Math.pow(alpha, 2) - (3.0 / 2.0) *
                Math.pow(alpha, 3) + (3.0 / 2.0) * Math.pow(alpha, 4) -
                0.5 * Math.pow(alpha, 5);
        double dpp1 = 0.5 * Math.pow(alpha, 3) - Math.pow(alpha, 4) +
                0.5 * Math.pow(alpha, 5);

        double x = p0 * points[0].x + p1 * points[1].x + dp0 * points[2].x + dp1 * points[3].x
                + dpp0 * points[4].x + dpp1 * points[5].x;
        double y = p0 * points[0].y + p1 * points[1].y + dp0 * points[2].y + dp1 * points[3].y
                + dpp0 * points[4].y + dpp1 * points[5].y;

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
        double p0 = -30 * Math.pow(alpha, 2) + 60 * Math.pow(alpha, 3) -
                30 * Math.pow(alpha, 4);
        double p1 = 30 * Math.pow(alpha, 2) - 60 * Math.pow(alpha, 3) +
                30 * Math.pow(alpha, 4);
        double dp0 = 1 - 18 * Math.pow(alpha, 2) + 32 * Math.pow(alpha, 3) -
                15 * Math.pow(alpha, 4);
        double dp1 = -12 * Math.pow(alpha, 2) + 28 * Math.pow(alpha, 3) -
                15 * Math.pow(alpha, 4);
        double ddp0 = alpha - (9.0 / 2) * Math.pow(alpha, 2) + 6 *
                Math.pow(alpha, 3) - (5.0 / 2) * Math.pow(alpha, 4);
        double ddp1 = (3.0 / 2) * Math.pow(alpha, 2) - 4 * Math.pow(alpha, 3) +
                (5.0 / 2) * Math.pow(alpha, 4);

        double dx = points[0].x * p0 + points[1].x * p1 + points[2].x * dp0 +
                points[3].x * dp1 + points[4].x * ddp0 + points[5].x * ddp1;

        double dy = points[0].y * p0 + points[1].y * p1 + points[2].y * dp0 +
                points[3].y * dp1 + points[4].y * ddp0 + points[5].y * ddp1;

        Vector d = new Vector(dx, dy);
        return d;
    }

    /**
     * @param from a point on the curve.
     * @param to another point on the curve.
     * @return the arc length between these two points.
     */
    @Override
    public double integrate(double from, double to) {
        //uses 3 point Gauss Quadrature method to approximate arc length: ∫a,b f(x)dx = Σi=0,n Ci*f(xi)
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
