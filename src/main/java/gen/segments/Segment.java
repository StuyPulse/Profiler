/**
 * Spline.java
 *
 * represents one part of a spline
 * spline is a picewise equation, a segment is one of those equations
 *
 * usually defined by start point, end point, and some tangents (heading) at those points
 */

package gen.segments;

import gen.Vector;
import gen.Waypoint;

import java.util.Objects;

public abstract class Segment {

    /**
     * a segment factory is used to generate splines of a certain type
     */
    public interface SegmentFactory {

        Segment getInstance(double tightness, Waypoint startwp, Waypoint endwp);

    }

    public final Vector[] points;

    /**
     * @param n order of spline
     * @param points points used to define spline
     */
    public Segment(int n, Vector... points) {
        if(points.length != n+1) throw new IllegalArgumentException("Incorrect number of points for segment");
        this.points = new Vector[points.length];
        for(int i = 0; i < points.length; i++) {
            this.points[i] = points[i];
        }
    }

    /**
     * @param alpha spline parameter from [0, 1]
     * @return point on the spline
     */
    public abstract Vector getCors(double alpha);

    /**
     * @param alpha spline parameter from [0, 1]
     * @return derivative (slope) at point as (x, y) vector
     */
    public abstract Vector differentiateVector(double alpha);

    /**
     * @param alpha spline parameter from [0, 1]
     * @return derivative (slope) at points as angle (heading)
     */
    public double differentiateAngle(double alpha) {
        Vector d = differentiateVector(alpha);
        double h = Math.atan2(d.y, d.x);
        h = (2 * Math.PI + h) % (2 * Math.PI);
        return h;
    }

    /**
     * @param from spline parameter of point a
     * @param to spline parameter of point b
     * @return arc length between a and b
     */
    public abstract double integrate(double from, double to);

    @Override
    public String toString() {
        String s = "";
        for(Vector point : points) s += point.toString() + " ";
        return s;
    }

}
