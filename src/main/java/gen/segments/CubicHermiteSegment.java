package gen.segments;

import gen.Vector;
import gen.Waypoint;

public class CubicHermiteSegment extends Segment {

    public static class CubicHermiteSegmentFactory implements Segment.SegmentFactory {

        @Override
        public CubicHermiteSegment getInstance(double tightness, Waypoint startwp, Waypoint endwp) {
            //scaling is just like with bezier curves except that it is x3 explanation can be found below:
            //http://www2.cs.uregina.ca/~anima/408/Notes/Interpolation/BezierDerivation.htm
            double scale = startwp.distanceTo(endwp) / 2 * tightness * 3;

            Vector[] points = new Vector[4];
            points[0] = startwp;
            points[1]= endwp;

            //atan2(dy, dx) = heading
            //so, cos(heading) * scale = dx and sin(heading) * scale = dy
            points[2] = new Vector(Math.cos(startwp.heading) * scale, Math.sin(startwp.heading) * scale);
            points[3] = new Vector(Math.cos(endwp.heading) * scale, Math.sin(endwp.heading) * scale);
            return new CubicHermiteSegment(points);
        }

    }

    public CubicHermiteSegment(Vector... points) {
        super(3, points);
    }

    @Override
    public Waypoint getWaypoint(double alpha) {
        double h1 = 2 * Math.pow(alpha, 3) - 3 * Math.pow(alpha, 2) + 1;
        double h2 = -2 * Math.pow(alpha, 3) + 3 * Math.pow(alpha, 2);
        double h3 = Math.pow(alpha, 3) - 2 * Math.pow(alpha, 2) + alpha;
        double h4 = Math.pow(alpha, 3) - Math.pow(alpha, 2);
        double x = points[0].x * h1 + points[1].x * h2 +
                points[2].x * h3 + points[3].x * h4;
        double y = points[0].y * h1 + points[1].y * h2 +
                points[2].y * h3 + points[3].y * h4;
        Vector d = differentiate(alpha);
        double h = Math.atan2(d.y, d.x);
        h = (2 * Math.PI + h) % (2 * Math.PI);
        Waypoint point = new Waypoint(x, y, h);
        point.distanceFromStart = integrate(0, alpha);
        point.distanceFromEnd = integrate(alpha, 1);
        return point;
    }

    @Override
    public Vector differentiate(double alpha) {
        // f'(x) = 3at^2 + 2b + c (using the power rule)
        double h1 = 6 * Math.pow(alpha, 2) - 6 * alpha;
        double h2 = -6 * Math.pow(alpha, 2) + 6 * alpha;
        double h3 = 3 * Math.pow(alpha, 2) - 4 * alpha + 1;
        double h4 = 3 * Math.pow(alpha, 2) - 2 * alpha;
        double dx = points[0].x * h1 + points[1].x * h2 +
                points[2].x * h3 + points[3].x * h4;
        double dy = points[0].y * h1 + points[1].y * h2 +
                points[2].y * h3 + points[3].y * h4;
        return new Vector(dx, dy);
    }

    @Override
    public double integrate(double from, double to) {
        //uses 2 point Gauss Quadrature method to approximate arc length: ∫a,b f(x)dx = Σi=0,n Ci*f(xi)
        //https://pomax.github.io/bezierinfo/#arclength
        //https://www.youtube.com/watch?v=unWguclP-Ds&feature=BFa&list=PLC8FC40C714F5E60F&index=1
        //values found here: https://pomax.github.io/bezierinfo/legendre-gauss.html
        double[] weights = {1.0000000000000000, 1.0000000000000000};
        double[] abscissa = {-0.5773502691896257, 0.5773502691896257};
        double sum = 0;
        for(int i = 0; i < 2; i++) {
            double pt = ((to - from) / 2.0) * abscissa[i] + ((to + from) / 2.0);
            Vector d = differentiate(pt);
            sum += weights[i] * Math.sqrt(d.x*d.x + d.y*d.y);
        }
        return ((to - from) / 2.0) * sum;
    }

}
