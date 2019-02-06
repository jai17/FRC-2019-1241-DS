package frc.robot.loops;

import frc.robot.subsystems.Carriage;

public class CarriageLoop implements Loop{

    private static CarriageLoop mInstance; 
		private Carriage carriage; 
	
    //The roller would be fixed to all control modes 
    public enum CarriageControlState {
		OPEN_LOOP, // open loop voltage control
    }
    
    private CarriageControlState mControlState = CarriageControlState.OPEN_LOOP;


    public static CarriageLoop getInstance() {
		if(mInstance == null){
			mInstance = new CarriageLoop();
			return mInstance;
		}
		else
			return mInstance;
    }

    private CarriageLoop() {
			carriage = Carriage.getInstance();
    }
    
    @Override
	public void onStart(double time_stamp) {
		System.out.println("Carriage Loop Started");
    }
    
    @Override
	public void onLoop(double time_stamp) {
		switch (mControlState) {
		case OPEN_LOOP:
			return;
		}
    }
    
    @Override
	public void onStop(double time_stamp) {
		// TODO Auto-generated method stub

    }
    
    public void setCarriageState(CarriageControlState state) {
		mControlState = state;
	}

	public CarriageControlState getControlState() {
		return mControlState;
	}
    
}