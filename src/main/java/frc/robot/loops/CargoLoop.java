package frc.robot.loops;

import frc.robot.subsystems.Cargo;

public class CargoLoop implements Loop{

    private static CargoLoop mInstance; 
		private Cargo cargo; 
	
    //The roller would be fixed to all control modes 
    public enum CargoControlState {
		OPEN_LOOP, // open loop voltage control
		MOTION_MAGIC, // motion magic pivot control
		PID_SETPOINT, // stock pid control
    }
    
    private CargoControlState mControlState = CargoControlState.OPEN_LOOP;


    public static CargoLoop getInstance() {
		if(mInstance == null){
			mInstance = new CargoLoop();
			return mInstance;
		}
		else
			return mInstance;
    }

    private CargoLoop() {
			cargo = Cargo.getInstance();
    }
    
    @Override
	public void onStart(double time_stamp) {
		System.out.println("Cargo Loop Started");
    }
    
    @Override
	public void onLoop(double time_stamp) {
		switch (mControlState) {
		case OPEN_LOOP:
			return;
		case MOTION_MAGIC:
			return;
		case PID_SETPOINT:
			return;
		}
    }
    
    @Override
	public void onStop(double time_stamp) {
		// TODO Auto-generated method stub

    }
    
    public void setCargoState(CargoControlState state) {
		mControlState = state;
	}

	public CargoControlState getControlState() {
		return mControlState;
	}
    
}