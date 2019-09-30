package gen.segments;

import gen.Vector;
import gen.Waypoint;

import java.util.Objects;

/**
 * @author Tahsin Ahmed
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
         *                  control points are to the start/end of
         *                  the individual curve.
         * @param startwp the starting waypoint.
         * @param endwp the ending waypoint
         * @return A segment that connects the starting waypoint to the
         * ending waypoint with the given tightness which
         * will stay constant throughout the generation of the curve.
         */
        Segment getInstance(double tightness, Waypoint startwp, Waypoint endwp);
    }

    /** The control and starting points. */
    public final Vector[] points;

    /**
     * @param n the order of the segment.
     * @param points the control and starting points.
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
     * @param alpha progression on curve.
     * @return the point on a curve (in vector form).
     */
    public abstract Vector getPoint(double alpha);

    /**
     * @param alpha progression on curve.
     * @return a Vector that contains the x and y
     * components of the slope of the point.
     */
    public abstract Vector differentiateC(double alpha);

    /**
     * @param alpha progression on curve.
     * @return the angle with respect to the x-axis of the point.
     */
    public double differentiateP(double alpha) {
        Vector d = differentiateC(alpha);
        double h = Math.atan2(d.y, d.x);
        h = (2 * Math.PI + h) % (2 * Math.PI);
        return h;
    }

    /**
     * @param from a point on the curve.
     * @param to another point on the curve.
     * @return the arclength between these two points.
     */
    public abstract double integrate(double from, double to);

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

    @Override
    public boolean equals(Object o) {
        if (o.getClass().equals(this.getClass())) {
            Segment s = (Segment) o;
            if (this.points.length != s.points.length) {
                return false;
            }
            for (int i = 0; i < points.length; i++) {
                if (!this.points[i].equals(s.points[i])) {
                    return false;
                }
            }
            return true;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(points);
    }

}
