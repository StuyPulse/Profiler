package followers;
import io.CSV;
import gen.Waypoint;

import java.io.*;

public class DistanceFollower {
    private double kp, ki, kd, kv, ka;
    private double previousError, heading;
    private int currentPoint;
    private Waypoint[] targetPoints;
    private double dt;

    public DistanceFollower(File csvData) {
        targetPoints = CSV.importCSV(csvData);
    }

    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.kv = kv; this.ka = ka;
    }

    private void reset() {
        previousError = 0;
        currentPoint = 0;
    }

    public double update(double distanceCovered) {
        dt = targetPoints[1].time;
        if(targetPoints.length > currentPoint) {
            Waypoint point = getWaypoint();
            double distanceError = point.distanceFromStart - distanceCovered;
            double calculated_value =
                    kp * distanceError + kd * ((distanceError - previousError) / dt) +
                            (kv * point.velocity + ka * point.acceleration);
            previousError = distanceError;
            currentPoint++;
            return calculated_value;
        } else return 0;
    }

    public double getHeading() {
        return targetPoints[currentPoint].heading;
    }

    public Waypoint getWaypoint() {
        return targetPoints[currentPoint];
    }

    public boolean isFinished() {
        return currentPoint >= targetPoints.length;
    }

}
