package gen;

import java.util.ArrayList;

public abstract class Trajectory {

    protected ArrayList<Waypoint> traj;

    public ArrayList<Waypoint> getPoints() {
        try {
            ArrayList<Waypoint> clone = new ArrayList<>(traj.size());
            for (Waypoint waypoint : traj) {
                clone.add((Waypoint) waypoint.clone());
            }
            return clone;
        } catch (Exception e) {
            System.out.println("error in copying");
            return null;
        }
    }

    /**
     * @author Tahsin Ahmed
     *
     * Types of ways to generate splines.
     */
    public enum FitMethod {

        CUBIC_BEZIER("cubic bezier"), CUBIC_HERMITE("cubic hermite"),
        QUINTIC_HERMITE("quintic hermite"), QUINTIC_BEZIER("quintic bezier");

        private String method;

        /**
         * @param method name of method.
         */
        FitMethod(String method) {
            this.method = method;
        }

        /**
         * @param str name of method in String format.
         * @return method as an enum.
         */
        public static CenterTrajectory.FitMethod findMethod(String str) {
            for (CenterTrajectory.FitMethod method : CenterTrajectory.FitMethod.values()) {
                if (method.getMethod().equals(str.toLowerCase())) return method;
            }
            return null;
        }

        public String getMethod() {
            return method;
        }

        @Override
        public String toString() {
            return method;
        }

    }

    /**
     * @author Tahsin Ahmed
     *
     * Some pre-defined values for sample rate of spline.
     */
    public enum SampleRate {

        LOW(1000), MEDIUM(10000), HIGH(100000);

        private int rate;

        /**
         * @param rate sample rate
         */
        SampleRate(int rate) {
            this.rate = rate;
        }

        public int getRate() {
            return rate;
        }

        @Override
        public String toString() {
            return Integer.toString(rate);
        }

    }

}
