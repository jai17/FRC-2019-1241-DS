package frc.robot.loops;

import frc.robot.subsystems.Hatch;

public class HatchLoop implements Loop{

    private static HatchLoop mInstance; 
		private Hatch hatch; 
	
    //The roller would be fixed to all control modes 
    public enum HatchControlState {
		OPEN_LOOP, // open loop voltage control
		MOTION_MAGIC, // motion magic pivot control
		PID_SETPOINT, // stock pid control
    }
    
    private HatchControlState mControlState = HatchControlState.OPEN_LOOP;


    public static HatchLoop getInstance() {
		if(mInstance == null){
			mInstance = new HatchLoop();
			return mInstance;
		}
		else
			return mInstance;
    }

    private HatchLoop() {
			hatch = Hatch.getInstance();
    }
    
    @Override
	public void onStart(double time_stamp) {
		System.out.println("Hatch Loop Started");
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
    
    public void setHatchState(HatchControlState state) {
		mControlState = state;
	}

	public HatchControlState getControlState() {
		return mControlState;
	}
    
}