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
            double scale = startwp.distanceTo(endwp) / 2 * tightness * 3;

            Vector[] points = new Vector[6];
            points[0] = startwp;
            points[1] = endwp;

            points[2] = new Vector(Math.cos(startwp.heading) * scale, Math.sin(startwp.heading) * scale);
            points[3] = new Vector(Math.cos(endwp.heading) * scale, Math.sin(endwp.heading) * scale);

            points[4] = new Vector(0, 0);
            points[5] = new Vector(0, 0);

            return new QuinticHermiteSegment(points)
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
        double f0 = 1 - 10 * Math.pow(alpha, 3) + 15 *
                Math.pow(alpha, 4) - 6 * Math.pow(alpha, 5);
        double f1 = alpha - 6 * Math.pow(alpha, 3) +  8 *
                Math.pow(alpha, 4) - 3 * Math.pow(alpha, 5);
        double f2 = (1.0 / 2.0) * Math.pow(alpha, 2) - (3.0 / 2.0) *
                Math.pow(alpha, 3) + (3.0 / 2.0) * Math.pow(alpha, 4) -
                (1.0 / 2.0) * Math.pow(alpha, 5);
        double f3 = (1.0 / 2.0) * Math.pow(alpha, 3) - Math.pow(alpha, 4) +
                (1.0 / 2.0) * Math.pow(alpha, 5);
        double f4 = -4 * Math.pow(alpha, 3) + 7 * Math.pow(alpha, 4) - 3 *
                Math.pow(alpha, 5);
        double f5 = 10 * Math.pow(alpha, 3) - 15 * Math.pow(alpha, 3) -
                15 * Math.pow(alpha, 4) + 6 * Math.pow(alpha, 5);

        double x = f0 * points[0].x + f1 * points[2].x + f2 * points[4].x + points[5].x *
                f3 + points[3].x * f4 + points[1].x * f5;
        double y = f0 * points[0].y + f1 * points[2].y + f2 * points[4].y + points[5].y *
                f3 + points[3].y * f4 + points[1].y * f5;

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
        double f0 = -30 * Math.pow(alpha, 2) + 60 * Math.pow(alpha, 3) -
                30 * Math.pow(alpha, 4);
        double f1 = 1 - 18 * Math.pow(alpha, 2) + 32 * Math.pow(alpha, 3) -
                15 * Math.pow(alpha, 4);
        double f2 = alpha - (9.0 / 2) * Math.pow(alpha, 2) + 6 *
                Math.pow(alpha, 3) - (5.0 / 2) * Math.pow(alpha, 4);
        double f3 = (3.0 / 2) * Math.pow(alpha, 2) - 4 * Math.pow(alpha, 3) +
                (5.0 / 2) * Math.pow(alpha, 4);
        double f4 = -12 * Math.pow(alpha, 2) + 28 * Math.pow(alpha, 3) -
                15 * Math.pow(alpha, 4);
        double f5 = -30 * Math.pow(alpha, 2) - 60 * Math.pow(alpha, 3) +
                30 * Math.pow(alpha, 4);

        double dx = points[0].x * f0 + points[1].x * f5 + points[2].x * f1 +
                points[3].x * f4 + points[4].x * f2 + points[5].x *  f3;

        double dy = points[0].y * f0 + points[1].y * f5 + points[2].y * f1 +
                points[3].y * f4 + points[4].y * f2 + points[5].y *  f3;

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
        final double[] WEIGHTS = {0.8888888888888888, 0.5555555555555556, 0.5555555555555556};
        final double[] ABSCISSA = {0.0000000000000000, -0.7745966692414834, 0.7745966692414834};
        double sum = 0;
        for (int i = 0; i < 3; i++) {
            double transformedPt = ((to - from) / 2.0) * ABSCISSA[i] + ((to + from) / 2.0);
            Vector d = differentiateVector(transformedPt);
            sum += WEIGHTS[i] * Math.sqrt(d.x * d.x + d.y * d.y);
        }
        return ((to - from) / 2.0) * sum;
    }
}
