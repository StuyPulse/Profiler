package gen.segments;

import gen.Vector;
import gen.Waypoint;

/**
 * Spline.java
 *
 * @author Tahsin Ahmed
 *
 * A template class used to represent different kinds of
 * extremely small parts (segments) of a curve.
 */


public abstract class Segment {

    /**
     *  An interface that allows a spline to use different kinds of segments.
     */
    public interface SegmentFactory {
        /**
         * @param tightness A multiplier that controls how close the
         *                  derivatives are to the start/end of
         *                  the individual curve.
         * @param startwp the starting waypoint.
         * @param endwp the ending waypoint
         * @return A segment that connects the starting waypoint to the
         * ending waypoint with the given tightness which
         * will stay constant throughout the generation of the curve.
         */
        Segment getInstance(double tightness, Waypoint startwp, Waypoint endwp);
    }

    /** The points used to define the spline. */
    public final Vector[] points;

    /**
     * @param angle to bound
     * @return equivalent angle between -180 to 180
     */
    private double boundToHalfRev(double angle) {
        if(angle <= Math.PI) return angle;
        else                 return angle - 2*Math.PI;
    }

    /**
     * @param n Order of spline
     * @param points Points used to define spline
     */
    public Segment(int n, Vector... points) {
        if (points.length != n + 1) {
            throw new IllegalArgumentException(
                    "Incorrect number of points for segment");
        }
        this.points = new Vector[points.length];
        for (int i = 0; i < points.length; i++) {
            this.points[i] = points[i];
        }
    }

    /**
     * @param alpha spline parameter from [0, 1]
     *              indicates progression on curve.
     * @return point on the spline.
     */
    public abstract Vector getCors(double alpha);

    /**
     * @param alpha spline parameter from [0, 1]
     *              indicates progression on curve.
     * @return first derivative (slope) at point
     *              as (x, y) vector.
     */
    public abstract Vector differentiate(double alpha);

    /**
     * @param alpha spline parameter from [0, 1]
     *              indicates progression on curve.
     * @return second derivative (slope of slope)
     *              as (x, y) vector.
     */
    public abstract Vector differentiateS(double alpha);

    /**
     * @param from point on curve.
     * @param to another point on curve.
     * @return arc length distance between points.
     */
    public abstract double integrate(double from, double to);

    /**
     * @param alpha spline parameter from [0, 1]
     *              indicates progression on curve.
     * @return curvature (k) indicating smoothness of path.
     *          same as angular velocity (w).
     */
    public double curvature(double alpha) {
        Vector d = differentiate(alpha);
        Vector dd = differentiateS(alpha);
        // k = ( x'(t)*y''(t) - y'(t)*x''(t) ) / (x'(t)^2 + y'(t)^2)^1.5
        return (d.x*dd.y - d.y*dd.x) / ( (d.x*d.x + d.y*d.y) * Math.sqrt(d.x*d.x + d.y*d.y) );
    }

    /**
     * @return a string representing all the points of the curve.
     */
    @Override
    public String toString() {
        String s = "";
        for (Vector point : points) {
            s += point.toString() + " ";
        }
        return s;
    }

}
