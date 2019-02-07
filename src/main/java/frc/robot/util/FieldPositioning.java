package frc.robot.util;

/**
 * FieldPositioning
 * @author Neil
 * 11/22/2018
 * All code related to field-relative positioning and point to point driving.
 */
public class FieldPositioning {

	/**
	 * Calculate the goal yaw setpoint based on the current and goal positions
	 * @param current - the (x,y) position of the robot currently
	 * @param goal - the desired (x,y) position of the robot
	 * @return the yaw setpoint for the robot to face the goal point
	 */
	public static double calcGoalYaw(Point current, Point goal) {
		double dx = goal.getxPos() - current.getxPos();
		double dy = goal.getyPos() - current.getyPos();
		
		double goalYaw = 0;
		
		//if goal point lies on x or y axis (dx or dy equal to zero)
		if(dy == 0) { //if no change in y
			if(dx > 0) {
				goalYaw = 90; //dead right
			} else if (dx < 0) {
				goalYaw = -90; //dead left
			}
		} else if (dx == 0) { //if no change in x
			if(dy > 0) {
				goalYaw = 0; //dead ahead
			} else if (dy < 0) {
				goalYaw = 180; //dead behind
			}
		} else {
			if (dy < 0) { //point is behind you
				if (dx > 0) {
					goalYaw = 90 - Math.toDegrees(Math.atan(dy/dx)); //behind and right
				} else if (dy < 0) {
					goalYaw = -90 - Math.toDegrees(Math.atan(dy/dx)); //behind and left
				}
			} else { //anywhere else
				goalYaw = Math.toDegrees(Math.atan(dx/dy));
			}
		}
		return goalYaw;
	}
}