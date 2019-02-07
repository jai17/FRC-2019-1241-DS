package frc.robot.loops;

import javax.lang.model.util.ElementScanner6;

import frc.robot.subsystems.Carriage;

public class CarriageLoop implements Loop{

    private static CarriageLoop mInstance; 
		private Carriage carriage; 

		//Open loop constants
		private boolean isRetracted; //slider
		private boolean isEngaged; //ejector
		private boolean isClosed; //claw

		private double feederSpeed;
		private double shooterSpeed;
		private boolean isFeeding;
		private boolean isShooting;
	
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
		if (!isRetracted) //slider
			carriage.extendCarriage();
		else 
			carriage.retractCarriage();

		if (!isEngaged) //ejector
			carriage.ejectHatch();
		else
			carriage.retractEjector();

		if (!isClosed) //claw
			carriage.holdAndSecure();
		else
			carriage.prisonBreak();

		if (!isFeeding)
			carriage.feederOut(feederSpeed);
		else
			carriage.feederIn(feederSpeed);

		if (!isShooting)
			carriage.shootBackwards(shooterSpeed);
		else 
			carriage.shootForward(shooterSpeed);

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

	//carriage states 
	//set slider position
	public void setSliderPos(boolean val) {
		isRetracted = val;
	}

	//set claw position
	public void setClawPos(boolean val) {
		isClosed = val;
	}
 
	//set ejector pos
	public void setEjectorPos(boolean val) {
		isEngaged = val;
	}

	//set feeder speed
	public void setFeederSpeed(double val) {
		feederSpeed = val;
	}

	//set feeder direction
	public void setIsFeeding(boolean val) {
		isFeeding = val;
	}

	//set shooter speed
	public void setShooterSpeed(double val) {
		feederSpeed = val;
	}

	//set shooter direction
	public void setIsShooting(boolean val) {
		isShooting = val;
	}
}