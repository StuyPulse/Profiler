package gen;

public class Vector implements Cloneable {

    public double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double magnitude() {
        return Math.sqrt(x*x + y*y);
    }

    public double distanceTo(Vector that) {
        return Math.sqrt((this.x - that.x) * (this.x - that.x) + (this.y - that.y) * (this.y - that.y));
    }

    public Vector offset(double xOffset, double yOffset) {
        return new Vector(x + xOffset, y + yOffset);
    }

    public Vector offset(Vector v) {
        return offset(v.x, v.y);
    }

    public Vector multiply(double n) {
        return new Vector(x * n, y * n);
    }

    public Vector normalize() {
        return new Vector(x / magnitude(), y / magnitude());
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
