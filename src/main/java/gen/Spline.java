package gen;

import gen.segments.Segment;
import gen.segments.Segment.SegmentFactory;

import java.util.Objects;

public class Spline {

    private final Segment[] segments;
    // private SegmentFactory segFact;
    public final double tightness;

    public Spline(double tightness, SegmentFactory segFact, Waypoint... waypoints) {
        if(waypoints.length < 2) throw new IllegalArgumentException("Not enough points for a spline");
        this.tightness = tightness;
        // this.segFact = segFact;
        segments = new Segment[waypoints.length - 1];
        for(int i = 0; i < waypoints.length - 1; i++) {
            segments[i] = segFact.getInstance(tightness, waypoints[i], waypoints[i+1]);
        }
    }

    public Waypoint getWaypoint(double alpha) {
        double pps = 1.0 / segments.length; // percent per curve
        int seg = (int) Math.floor(alpha / pps);
        double a = (alpha % pps) / pps;
        return segments[seg].getWaypoint(a);
    }

    public Vector differentiate(double alpha) {
        double pps = 1.0 / segments.length; // percent per curve
        int seg = (int) Math.floor(alpha / pps);
        double a = (alpha % pps) / pps;
        return segments[seg].differentiate(a);
    }

    public double integrate(double from, double to) {
        double pps = 1.0 / segments.length; // percent per curve
        int seg = (int) Math.floor(from / pps);
        if(from - to == pps) {
            return segments[seg].integrate(0.0, 1.0);
        }else if(from - to < pps) {
            double a = (from % pps) / pps;
            double b = (to % pps) / pps;
            return segments[seg].integrate(a, b);
        }else {
            double a = (from % pps) / pps;
            double n = (seg+1) * pps;
            return segments[seg].integrate(a, 1.0) + integrate(n, to);
        }
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
    public int hashCode() { return Objects.hash(segments); }

}