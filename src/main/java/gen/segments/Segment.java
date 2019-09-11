package gen.segments;

import gen.Vector;
import gen.Waypoint;

import java.util.Objects;

public abstract class Segment {

    public interface SegmentFactory {

        Segment getInstance(double tightness, Waypoint startwp, Waypoint endwp);

    }

    public final Vector[] points;

    public Segment(int n, Vector... points) {
        if(points.length != n+1) throw new IllegalArgumentException("Incorrect number of points for segment");
        this.points = new Vector[points.length];
        for(int i = 0; i < points.length; i++) {
            this.points[i] = points[i];
        }
    }

    public abstract Waypoint getWaypoint(double alpha);

    public abstract Vector differentiate(double alpha);

    public abstract double integrate(double from, double to);

    @Override
    public String toString() {
        String s = "";
        for(Vector point : points) s += point.toString() + " ";
        return s;
    }

    @Override
    public boolean equals(Object o) {
        if(o.getClass().equals(this.getClass())) {
            Segment s = (Segment) o;
            if(this.points.length != s.points.length) return false;
            for(int i = 0; i < points.length; i++) {
                if(!this.points[i].equals(s.points[i])) return false;
            }
            return true;
        }else return false;
    }

    @Override
    public int hashCode() { return Objects.hash(points); }

}
