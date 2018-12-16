import java.util.ArrayList;

public class Follower {
	private ArrayList<Waypoint> trajectory;
	private int waypointNum; 
	private double dt; 
	private double kp, ki, eThreshold, kd, kv, ka; 
	private double currentError, lastError, errorNum; 
	
	public Follower(ArrayList<Waypoint> trajectory) {
		this.trajectory = trajectory;
		waypointNum = 0; 
		this.dt = trajectory.get(1).time;
		currentError = 0; 
		lastError = 0; 
		errorNum = 0; 
	}
	
	public void setTrajectory(ArrayList<Waypoint> trajectory) {
		this.trajectory = trajectory; 
		this.dt = trajectory.get(1).time; 
		reset(); 
	}
	
	public void reset() {
		waypointNum = 0;
		currentError = 0; 
		lastError = 0; 
		errorNum = 0; 
	}
	
	public void configureGains(double kp, double ki, double eThreshold, double kd, double kv, double ka) {
		this.kp = kp; 
		this.ki = ki; 
		this.eThreshold = Math.abs(eThreshold); 
		this.kd = kd; 
		this.kv = kv; 
		this.ka = ka; 
	}
	
	public Waypoint getCurrentWaypoint() {
		return trajectory.get(waypointNum); 
	}
	
	public double calculate(double distance) {
		currentError = getCurrentWaypoint().distanceFromStart - distance;
		if(Math.abs(currentError) >= eThreshold) errorNum++; 
		double p = kp * currentError; 
		double i = ki * errorNum; 
		double d = kd * ((currentError - lastError) / dt); 
		double v = kv * getCurrentWaypoint().velocity;
		double a = ka * getCurrentWaypoint().acceleration;
		if(!isLast()) waypointNum++; 
		return p + i + d + v + a; 
	}
	
	public boolean isLast() {
		return waypointNum >= trajectory.size() - 1; 
	}
}
