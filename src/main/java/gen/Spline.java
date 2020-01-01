package gen;
import gen.segments.Segment;
import gen.segments.Segment.SegmentFactory;

/**
 * Spline.java
 *
 * @author Tahsin Ahmed
 *
 * A spline is made up of a bunch of segments.
 */

public class Spline {

    /**
     * A position contains its segment id and its corresponding
     * progression on the curve.
     */
    private static class Position {
        /** An id that indicates the (segth) segment of the spline. */
        private final int seg;
        /** Progression on the curve [0, 1]. */
        private final double alpha;

        /**
         * @param seg an id that indicates the (segth) segment of the spline.
         * @param alpha progression on the curve [0, 1].
         */
        private Position(int seg, double alpha) {
            this.seg = seg;
            this.alpha = alpha;
        }

    }

    /** The segments of a spline. */
    private final Segment[] segments;
    /** A multiplier that controls how close the
     *  control points are to the start/end of
     *  the individual curve. */
    public final double tightness;

    /**
     * @param tightness A multiplier that controls how close the
     *                  control points are to the start/end of
     *                  the individual curve.
     * @param segFact the segment generator.
     * @param waypoints points that you want your curve to be on.
     */
    public Spline(double tightness, SegmentFactory segFact, Waypoint... waypoints) {
        if(waypoints.length < 2) throw new IllegalArgumentException("Not enough points for a spline");
        this.tightness = tightness;
        segments = new Segment[waypoints.length - 1];
        for(int i = 0; i < waypoints.length - 1; i++) {
            segments[i] = segFact.getInstance(tightness, waypoints[i], waypoints[i+1]);
        }
    }

    /**
     * @param alpha progression on spline [0, 1].
     * @return the segment and alpha of the segment corresponding to the spline alpha.
     */
    private Position findPoint(double alpha) {
        double percent = 1.0 / segments.length;
        int s = (int) Math.min(Math.max(Math.floor(alpha / percent), 0), segments.length-1);
        double a = (alpha - s * percent) / percent;
        return new Position(s, a);
    }

    /**
     * @param alpha progression on spline [0, 1].
     * @return position on the curve.
     */
    private Vector getPoint(double alpha) {
        Position pos = findPoint(alpha);
        return segments[pos.seg].getCors(pos.alpha);
    }

    /**
     * @param alpha progression on spline [0, 1].
     * @return heading at points in radians.
     */
    public double getHeading(double alpha) {
        Position pos = findPoint(alpha);
        Vector h = segments[pos.seg].differentiate(pos.alpha);
        return h.x >= 0 ? Math.atan(h.y / h.x) : Math.PI + Math.atan(h.y / h.x);
    }

    /**
     * @param alpha progression on the spline [0, 1].
     * @return angular velocity at that point in angle format.
     */
    public double getCurvature(double alpha) {
        Position pos = findPoint(alpha);
        return segments[pos.seg].curvature(pos.alpha);
    }

    /**
     * @param from start progression on curve.
     * @param to end progression on curve.
     * @return arclength from one point to another.
     */
    public double getDistance(double from, double to) {
        Position fPos = findPoint(from);
        Position tPos = findPoint(to);
        if (fPos.seg == tPos.seg)
            return segments[fPos.seg].integrate(fPos.alpha, tPos.alpha);
        else
            return segments[fPos.seg].integrate(fPos.alpha, 1.0) +
                    getDistance((fPos.seg + 1.0) / segments.length, to);
    }

    /**
     * @param alpha progression on the curve [0, 1].
     * @return a waypoint on the curve.
     */
    public Waypoint getWaypoint(double alpha) {
        Waypoint wp = new Waypoint(getPoint(alpha), getHeading(alpha));
        wp.distanceFromStart = getDistance(0, alpha);
        wp.distanceFromEnd   = getDistance(alpha, 1.0);
        wp.curvature         = getCurvature(alpha);
        return wp;
    }

    /**
     * @return the amount of segments in the curve.
     */
    public int size() {
        return segments.length;
    }

    /** Gets control points: Vectors that will define the curve. */
    public Waypoint[] getControlPoints() {
        Waypoint[] w = new Waypoint[size() + 1];
        for (int i = 0; i < size(); i++) {
            w[i] = (Waypoint) segments[i].points[0];
        }
        Segment last = segments[size() -1];
        w[size()] = (Waypoint) last.points[last.points.length - 1];
        return w;
    }

    @Override
    public String toString() {
        String s = "";
        for (Segment segment : segments) {
            s += segment.toString() + "\n";
        }
        return s;
    }

}
