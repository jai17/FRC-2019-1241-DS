package frc.robot.loops;

import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vector;

public class RobotPoseTracker implements Loop{
	
	private static RobotPoseTracker mInstance;
	Drivetrain drive = Drivetrain.getInstance();
	RobotState state = RobotState.getInstance();
	
	double left_encoder_prev_distance = 0;
	double right_encoder_prev_distance = 0;
	
	public static RobotPoseTracker getInstance() {
		if(mInstance == null){
			mInstance = new RobotPoseTracker();
			return mInstance;
		}
		else
			return mInstance;
	}

	@Override
	public void onStart(double timestamp) {
		left_encoder_prev_distance = drive.getLeftPos();
		right_encoder_prev_distance = drive.getRightPos();
		System.out.println("Starting Robot Tracker");
	}

	@Override
	public void onLoop(double timestamp) {
		double left = drive.getLeftPos();
		double right = drive.getRightPos();
		Vector velocity_delta = state.generateVectorFromSensors(left - left_encoder_prev_distance, right - right_encoder_prev_distance, drive.getYaw());
		Vector measured_velocity = state.generateVectorFromSensors(drive.getLeftVelocityInchesPerSec(), drive.getRightVelocityInchesPerSec(), drive.getYaw());
		state.updateState(velocity_delta, measured_velocity);
		left_encoder_prev_distance = left;
		right_encoder_prev_distance = right;
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub
		
	}

}