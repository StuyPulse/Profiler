package gen;

import java.util.ArrayList;

public class SideTrajectory extends Trajectory {

    public final Spline spline;
    public final double offset;
    public final double theta;

    public final int sampleRate;
    public final double dt;

    public final double maxVelocity, maxAcceleration, maxJerk;

    public SideTrajectory(Spline spline, double offset, double theta, int sampleRate, double dt,
                          double maxVelocity, double maxAcceleration, double maxJerk) {
        this.spline = spline;
        this.offset = offset;
        this.theta = theta;

        this.sampleRate = sampleRate;
        this.dt = dt;

        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;

        traj = new ArrayList<>();
        generate();
    }

    private void sample() {
        if(!traj.isEmpty()) traj.clear();
        int NUMBER_OF_POINTS = sampleRate * spline.size();
        for (int i = 0; i <= NUMBER_OF_POINTS; i++) {
            double a = (double) i / NUMBER_OF_POINTS;
            traj.add(spline.offsetWaypoint(a, offset, theta));
        }
    }

    private void generate() {
        sample();
    }

}
