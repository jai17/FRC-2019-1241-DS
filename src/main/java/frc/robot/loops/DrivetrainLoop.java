package frc.robot.loops;

import frc.robot.subsystems.Drivetrain;

public class DrivetrainLoop implements Loop{

    private static DrivetrainLoop mInstance; 
		private Drivetrain drive; 
		
		//Open Loop Constants
		private double openLoopLeft = 0; 
		private double openLoopRight = 0; 

    public enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		VELOCITY_SETPOINT, // velocity PID control
		PATH_FOLLOWING, // used for autonomous driving
		TURN_TO_HEADING
    }
    
    private DriveControlState mControlState = DriveControlState.OPEN_LOOP;


    public static DrivetrainLoop getInstance() {
		if(mInstance == null){
			mInstance = new DrivetrainLoop();
			return mInstance;
		}
		else
			return mInstance;
    }

    private DrivetrainLoop() {
			drive = Drivetrain.getInstance();
    }
    
    @Override
	public void onStart(double time_stamp) {
		System.out.println("Drive Loop Started");
    }
    
    @Override
	public void onLoop(double time_stamp) {
		switch (mControlState) {
		case OPEN_LOOP:
		drive.runLeftDrive(openLoopLeft);
		drive.runRightDrive(openLoopRight);
			return;
		case VELOCITY_SETPOINT:
			return;
		case PATH_FOLLOWING:
			return;
		case TURN_TO_HEADING:
			return;
		}
    }
    
    @Override
	public void onStop(double time_stamp) {
		// TODO Auto-generated method stub

    }
    
    public void setDriveState(DriveControlState state) {
		mControlState = state;
	}

	public void setLeftDrive(double val){
		this.openLoopLeft = val; 
	}
	public void setRightDrive(double val){
		this.openLoopRight = val; 
	}
	
	public DriveControlState getControlState() {
		return mControlState;
	}
    
}