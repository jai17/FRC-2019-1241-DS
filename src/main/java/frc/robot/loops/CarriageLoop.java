package frc.robot.loops;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.commands.hatch.HatchFeedSequence;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Carriage;

public class CarriageLoop implements Loop{

	private static CarriageLoop mInstance; 
	private Carriage carriage; 
	private Cargo cargo;

	//Open loop constants
	private boolean isRetracted = true; //slider
	private boolean isEngaged =  true; //ejector
	private boolean isClosed =  true; //claw

	private double feederSpeed = 0;
	private double shooterSpeed  = 0;
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
			cargo = Cargo.getInstance();
	}
	
	@Override
	public void onStart(double time_stamp) {
		System.out.println("Carriage Loop Started");
	}
	
	@Override
	public void onLoop(double time_stamp) {
		switch (mControlState) {
			case OPEN_LOOP:
			if (!DriverStation.getInstance().isAutonomous() && !HatchFeedSequence.getInstance().isRunning()){
				if (isRetracted) { //slider
					carriage.extendCarriage();
					// System.out.println("EXTEND CARRIAGE");
				} else {
					carriage.retractCarriage();
				}

				if (!isEngaged) { //ejector
					carriage.ejectHatch();
					// System.out.println("EJECT HATCH");
				} else {
					carriage.retractEjector();
				}

				if (!isClosed) { //claw
					carriage.holdAndSecure();
					// System.out.println("CLOSE CLAW");
				} else {
					carriage.prisonBreak();
				}
			}

			if (!carriage.getOptic() && isShooting) { //if ball in and is shooting
				carriage.feederIn(1);
			} 
			
			else if (!carriage.getOptic()) { //if ball in
				this.setFeederSpeed(0);
				carriage.feederOut(0);
			}
			else {
				carriage.feederIn(feederSpeed);
			}

			if (isShooting) { //shooter
				carriage.shootBackwards(shooterSpeed);
			} else {
				carriage.shootForward(0);
			}

				return; //end OPEN_LOOP
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

	//get slider position
	public boolean getSliderPos() {
		return isRetracted;
	}

	//set claw position
	public void setClawPos(boolean val) {
		isClosed = val;
	}

	//get claw position
	public boolean getClawPos() {
		return isClosed;
	}

	//set ejector pos
	public void setEjectorPos(boolean val) {
		isEngaged = val;
	}

	//get ejector position
	public boolean getEjectorPos() {
		return isEngaged;
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
		shooterSpeed = val;
	}

	//set shooter direction
	public void setIsShooting(boolean val) {
		isShooting = val;
	}
}