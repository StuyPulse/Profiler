package gen;

import gen.segments.Segment;
import gen.segments.Segment.SegmentFactory;

import java.util.Objects;

/**
 *
 */
public class Spline {

    private static class Position {

        private final int seg;
        private final double alpha;

        private Position(int seg, double alpha) {
            this.seg = seg;
            this.alpha = alpha;
        }

    }

    private final Segment[] segments;
    public final double tightness;

    public Spline(double tightness, SegmentFactory segFact, Waypoint... waypoints) {
        if(waypoints.length < 2) throw new IllegalArgumentException("Not enough points for a spline");
        this.tightness = tightness;
        segments = new Segment[waypoints.length - 1];
        for(int i = 0; i < waypoints.length - 1; i++) {
            segments[i] = segFact.getInstance(tightness, waypoints[i], waypoints[i+1]);
        }
    }

    private Position findPoint(double alpha) {
        double percent = 1.0 / segments.length;
        int s = (int) Math.min(Math.max(Math.floor(alpha / percent), 0), segments.length-1);
        double a = (alpha - s * percent) / percent;
        return new Position(s, a);
    }

    private Vector getPoint(double alpha) {
        Position pos = findPoint(alpha);
        // TODO null pointer exception
        return segments[pos.seg].getPoint(pos.alpha);
    }

    public Vector headingC(double alpha) {
        Position pos = findPoint(alpha);
        return segments[pos.seg].differentiateC(pos.alpha);
    }

    public double headingP(double alpha) {
        Position pos = findPoint(alpha);
        return segments[pos.seg].differentiateP(pos.alpha);
    }

    public double distance(double from, double to) {
        Position fPos = findPoint(from);
        Position tPos = findPoint(to);
        if (fPos.seg == tPos.seg) {
            return segments[fPos.seg].integrate(fPos.alpha, tPos.alpha);
        } else {
            return segments[fPos.seg].integrate(fPos.alpha, 1.0) + distance(fPos.seg + 1, to);
        }
    }

    // TODO create lite version of functions above to speed up this process

    public Waypoint getWaypoint(double alpha) {
        Waypoint wp = new Waypoint(getPoint(alpha), headingP(alpha));
        wp.distanceFromStart = distance(0, alpha);
        wp.distanceFromEnd   = distance(alpha, size());
        return wp;
    }

    public int size() { return segments.length; }

    public Waypoint[] waypoints() {
        Waypoint[] w = new Waypoint[size() + 1];
        for(int i = 0; i < size(); i++) {
            w[i] = (Waypoint) segments[i].points[0];
        }
        Segment last = segments[size() -1];
        w[size()] = (Waypoint) last.points[last.points.length - 1];
        return w;
    }

    @Override
    public String toString() {
        String s = "";
        for(Segment segment : segments) s += segment.toString() + "\n";
        return s;
    }

    @Override
    public boolean equals(Object o) {
        if(o.getClass().equals(this.getClass())) {
            Spline s = (Spline) o;
            if(this.segments.length != s.segments.length) return false;
            for(int i = 0; i < segments.length; i++) {
                if(!this.segments[i].equals(s.segments[i])) return false;
            }
            return true;
        }else return false;
    }

    @Override
    public int hashCode() { return Objects.hash(segments, tightness); }

}