package gen;

import java.util.Objects;

public class Vector {

    public double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector PolarPoint(double distance, double rotation) {
        double x = Math.cos(rotation) * distance;
        double y = Math.sin(rotation) * distance;
        return new Vector(x, y);
    }

    public double distanceTo(Waypoint that) {
        return Math.sqrt((this.x - that.x) * (this.x - that.x) + (this.y - that.y) * (this.y - that.y));
    }

    public Vector clone() {
        return new Vector(this.x, this.y);
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    @Override
    public boolean equals(Object o) {
        if (o.getClass().equals(this.getClass())) {
            Vector v = (Vector) o;
            return x == v.x && y == v.y;
        } else return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

}
