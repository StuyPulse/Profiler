package Generation;
import java.util.ArrayList;

public class SideTraj extends CenterTraj {
	private CenterTraj centerTraj;
	private double offset; 

	public SideTraj(CenterTraj centerTraj, double offset) {
		super(centerTraj.method, 
				centerTraj.sampleRate, 
				centerTraj.dt, 
				centerTraj.wheelBaseWidth, 
				centerTraj.maxVel, 
				centerTraj.maxAccel, 
				centerTraj.maxJerk, 
				centerTraj.waypoints);
		this.centerTraj = centerTraj; 
		this.offset = offset; 
	}
	
	private void genOffsetPath() {
		ArrayList<Waypoint> centerPath = centerTraj.getPath();
		for(int i = 0; i < centerPath.size(); i++) {
			Waypoint point = centerPath.get(i).offsetWaypointPerpen(offset);
			point.time = centerPath.get(i).time;
			traj.add(point);
		}
	}
	
	@Override 
	protected void getVelocities() {
		ArrayList<Waypoint> centerPath = centerTraj.getPath();
		//Iterating through the points of the trajectory
		for(int i = 0 ; i < centerPath.size(); i ++) {
		//Find the difference the sides have to be to be spinning at that angular velocity
			double velocityDifference = (wheelBaseWidth / 2) * centerPath.get(i).angularVelocity;
			if(traj.get(i).distanceFromStart > centerPath.get(i).distanceFromStart) {
				//If side is outside it is greater
				traj.get(i).velocity = centerPath.get(i).velocity + velocityDifference;  
			}else if(traj.get(i).distanceFromStart < centerPath.get(i).distanceFromStart) {
				//If side is outside it is greater
				traj.get(i).velocity = centerPath.get(i).velocity - velocityDifference; 
			}else {
				//Otherwise its all equal and straight
				traj.get(i).velocity = centerPath.get(i).velocity; 
			}
		}		
	}
	
	@Override 
	public void generate() {
		//Path values
		genOffsetPath();
		getDistancesFromStart();
		getHeadings();
						
		//Velocity and velocity corrections
		getVelocities(); 
			
		//Accelerations and jerk
		getAccelerations();  
		getJerks(); //60 ft/sec^3 best for trapezodial motion profile
	}
}
