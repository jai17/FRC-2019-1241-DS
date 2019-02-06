package frc.robot; 

import frc.robot.util.Vector; 

public class RobotState {
	private static RobotState mInstance;
	
	private Vector field_to_robot = new Vector(0,0);
	private Vector velocity_delta;
	private Vector measured_velocity;
	private double distance_driven = 0;

	public static RobotState getInstance() {
		if(mInstance == null){
			mInstance = new RobotState();
			return mInstance;
		}
		else
			return mInstance;
	}
    
    
    public void updateState(Vector velocity_delta, Vector measured_velocity) {
    	field_to_robot = field_to_robot.add(velocity_delta);
    	this.velocity_delta = velocity_delta;
    	this.measured_velocity = measured_velocity;
    }
    
    public Vector generateVectorFromSensors(double left_delta, double right_delta, double angle) {
    	double r = (left_delta + right_delta)/2.0;
    	Vector delta = new Vector(r, angle);
    	distance_driven += delta.getR();
    	return delta;
    }
    
    public Vector getFieldToRobot() {
    	return field_to_robot;
    }
    
    public Vector getCurrentVelocity() {
    	return measured_velocity;
    }
    
    public Vector getVelocityDelta() {
    	return velocity_delta;
    }
    
    public double getDistanceDriven() {
    	return distance_driven;
    }
}
