package gen.segments;

import gen.Vector;
import gen.Waypoint;

/**
 * CubicHermiteSegment.java
 *
 * @author Tahsin Ahmed
 *
 * A segment that is generated through the cubic hermite equations.
 */
public class CubicHermiteSegment extends Segment {

    /** A cubic segment generator used by the corresponding spline.  */
    public static class CubicHermiteSegmentFactory implements Segment.SegmentFactory {
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
        public CubicHermiteSegment getInstance(double tightness,
                                               Waypoint startwp, Waypoint endwp) {
            // scaling is just like with bezier curves except
            // that it is x3 explanation can be found below:
            // http://www2.cs.uregina.ca/~anima/408/Notes/Interpolation/BezierDerivation.htm
            double scale = startwp.distanceTo(endwp) / 2 * tightness * 3;

            Vector[] points = new Vector[4];
            points[0] = startwp;
            points[1] = endwp;

            // atan2(dy, dx) = heading
            // so, cos(heading) * scale = dx and sin(heading) * scale = dy
            points[2] = new Vector(Math.cos(startwp.heading) * scale, Math.sin(startwp.heading) * scale);
            points[3] = new Vector(Math.cos(endwp.heading) * scale, Math.sin(endwp.heading) * scale);

            return new CubicHermiteSegment(points);
        }

    }

    /**
     * 3 is the order of the segment.
     * @param points the control and starting points.
     */
    public CubicHermiteSegment(Vector... points) {
        super(3, points);
    }

    /**
     * @param alpha spline parameter [0, 1]
     *              indicates progression on curve.
     * @return the point on a curve (in vector form).
     */
    @Override
    public Vector getCors(double alpha) {
        double h1 = 2 * Math.pow(alpha, 3) - 3 * Math.pow(alpha, 2) + 1;
        double h2 = -2 * Math.pow(alpha, 3) + 3 * Math.pow(alpha, 2);
        double h3 = Math.pow(alpha, 3) - 2 * Math.pow(alpha, 2) + alpha;
        double h4 = Math.pow(alpha, 3) - Math.pow(alpha, 2);
        double x = points[0].x * h1 + points[1].x * h2 +
                points[2].x * h3 + points[3].x * h4;
        double y = points[0].y * h1 + points[1].y * h2 +
                points[2].y * h3 + points[3].y * h4;
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
        // f'(x) = 3at^2 + 2b + c (using the power rule)
        double h1 = 6 * Math.pow(alpha, 2) - 6 * alpha;
        double h2 = -6 * Math.pow(alpha, 2) + 6 * alpha;
        double h3 = 3 * Math.pow(alpha, 2) - 4 * alpha + 1;
        double h4 = 3 * Math.pow(alpha, 2) - 2 * alpha;
        double dx = points[0].x * h1 + points[1].x * h2 +
                points[2].x * h3 + points[3].x * h4;
        double dy = points[0].y * h1 + points[1].y * h2 +
                points[2].y * h3 + points[3].y * h4;
        return new Vector(dx, dy);
    }

    /**
     * @param from a point on the curve.
     * @param to another point on the curve.
     * @return the arc length between points.
     */
    @Override
    public double integrate(double from, double to) {
        // uses 2 point Gauss Quadrature method to approximate arc length: ∫a,b f(x)dx = Σi=0,n Ci*f(xi)
        // https://pomax.github.io/bezierinfo/#arclength
        // https://www.youtube.com/watch?v=unWguclP-Ds&feature=BFa&list=PLC8FC40C714F5E60F&index=1
        // values found here: https://pomax.github.io/bezierinfo/legendre-gauss.html
        final double[] WEIGHTS = {1.0000000000000000, 1.0000000000000000};
        final double[] ABSCISSA = {-0.5773502691896257, 0.5773502691896257};
        double sum = 0;
        for(int i = 0; i < 2; i++) {
            double pt = ((to - from) / 2.0) * ABSCISSA[i] + ((to + from) / 2.0);
            Vector d = differentiateVector(pt);
            sum += WEIGHTS[i] * Math.sqrt(d.x*d.x + d.y*d.y);

        }
        return ((to - from) / 2.0) * sum;
    }

}
