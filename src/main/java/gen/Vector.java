package gen;

public class Vector implements Cloneable {

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

    public double distanceTo(Vector that) {
        return Math.sqrt((this.x - that.x) * (this.x - that.x) + (this.y - that.y) * (this.y - that.y));
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    @Override
    public Object clone() throws CloneNotSupportedException {
        return super.clone();
    }

}
