package gen.modifiers;

import gen.Trajectory;
import gen.Vector;
import gen.Waypoint;

import java.util.ArrayList;

/**
 * TankModifier.java
 *
 * @author Tahsin Ahmed
 *
 * Modifies into left and right trajectory to be used for tank drive.
 */

public class TankModifier extends Modifier {

    private double offset;
    private ArrayList<Waypoint> left, right;

    /**
     * @param original the basis trajectory
     * @param offset 1/2 wheel base width
     */
    public TankModifier(Trajectory original, double offset) {
        super(original);
        this.offset = offset;
        genLeft();
        genRight();
    }

    /**Generate the left*/
    private void genLeft() {
        left = new ArrayList<>();
        double total_dist = 0;
        Waypoint last = null;
        for(Waypoint w : original.getPoints()) {
            Vector length = w.rotate(Math.PI / 2).direction().multiply(offset);
            Waypoint next = w.offset(length);

            next.time = w.time;

            total_dist += last == null ? 0 : next.distanceTo(last);
            next.distanceFromStart = total_dist;

            /*
             * positive curvature; turning counter-clockwise; left on inside; less velocity
             * negative curvature; turning clockwise; left on outside; greater velocity
             */
            next.velocity = w.velocity - (w.curvature * offset);

            next.acceleration = last == null ? 0 : (next.velocity - last.velocity) / (next.time - last.time);
            next.jerk = last == null ? 0 : (next.acceleration - last.acceleration) / (next.time - last.time);

            left.add(next);
            last = next;
        }
    }

    /**Get copy of left*/
    public ArrayList<Waypoint> getLeft() {
        try {
            ArrayList<Waypoint> clone = new ArrayList<>(left.size());
            for (Waypoint waypoint : left)
                clone.add((Waypoint) waypoint.clone());
            return clone;
        } catch (Exception e) {
            System.out.println("error in copying");
            return null;
        }
    }

    /**Generate the right*/
    private void genRight() {
        right = new ArrayList<>();
        double total_dist = 0;
        double total_time = 0;
        Waypoint last = null;
        for(Waypoint w : original.getPoints()) {
            Vector length = w.rotate(-Math.PI / 2).direction().multiply(offset);
            Waypoint next = w.offset(length);

            next.time = w.time;

            total_dist += last == null ? 0 : next.distanceTo(last);
            next.distanceFromStart = total_dist;

            /*
             * positive curvature; turning counter-clockwise; right on outside; greater velocity
             * negative curvature; turning clockwise; right on inside; less velocity
             */
            next.velocity = w.velocity + (w.curvature * offset);

            next.acceleration = last == null ? 0 : (next.velocity - last.velocity) / (next.time - last.time);
            next.jerk = last == null ? 0 : (next.acceleration - last.acceleration) / (next.time - last.time);

            right.add(next);
            last = next;
        }
    }

    /**Get copy of right*/
    public ArrayList<Waypoint> getRight() {
        try {
            ArrayList<Waypoint> clone = new ArrayList<>(right.size());
            for (Waypoint waypoint : right)
                clone.add((Waypoint) waypoint.clone());
            return clone;
        } catch (Exception e) {
            System.out.println("error in copying");
            return null;
        }
    }

}
