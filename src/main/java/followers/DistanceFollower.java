package followers;
import gen.Waypoint;
import java.util.*;
import java.io.*;

public class DistanceFollower extends Path {
    private double kp, ki, kd, kv, ka;
    private double last_error, heading;
    private int atPoint;
    private Waypoint[] targetPoints;
    private double dt;

    public DistanceFollower(String file) {
        super(new File(file));
        targetPoints = getTargetPoints();
        dt = getDt();
    }

    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.kv = kv; this.ka = ka;
    }

    private void reset() {
        last_error = 0;
        atPoint = 0;
    }

    public double calculate(double distance_covered) {
        if(targetPoints.length > atPoint) {
            Waypoint point = targetPoints[atPoint];
            double error = point.distanceFromStart - distance_covered;
            double calculated_value =
                    kp * error + kd * ((error - last_error) / dt) +
                            (kv * point.velocity + ka * point.acceleration);
            last_error = error;
            heading = point.heading;
            atPoint++;
            return calculated_value;
        } else return 0;
    }

    public double getHeading() {
        return heading;
    }

    public Waypoint getWaypoint() {
        return targetPoints[atPoint];
    }

    public boolean isFinished() {
        return atPoint >= targetPoints.length;
    }

}
