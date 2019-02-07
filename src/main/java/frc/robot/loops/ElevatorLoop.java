package frc.robot.loops;

import frc.robot.subsystems.Elevator;;

public class ElevatorLoop implements Loop {

	private static ElevatorLoop mInstance;
	private Elevator elevator;

	// Motion Magic Constants
	public double magicSetpoint;
	public int cruiseVelocity;
	public double secsToMaxSpeed;

	//Open Loop Constants
	public double openLoopSpeed; 

	// The roller would be fixed to all control modes
	public enum ElevatorControlState {
		OPEN_LOOP, // open loop voltage control
		MOTION_MAGIC, // motion magic setpoint control
		PID_SETPOINT, // stock pid control
	}

	private ElevatorControlState mControlState = ElevatorControlState.MOTION_MAGIC;

	public static ElevatorLoop getInstance() {
		if (mInstance == null) {
			mInstance = new ElevatorLoop();
			return mInstance;
		} else
			return mInstance;
	}

	private ElevatorLoop() {
		elevator = Elevator.getInstance();
	}

	@Override
	public void onStart(double time_stamp) {
		System.out.println("Elevator Loop Started");
	}

	@Override
	public void onLoop(double time_stamp) {
		switch (mControlState) {
		case OPEN_LOOP:
			elevator.runElevator(openLoopSpeed);
			return;
		case MOTION_MAGIC:
			elevator.setMotionMagicSetpoint(magicSetpoint, cruiseVelocity, secsToMaxSpeed);
			return;
		case PID_SETPOINT:
			return;
		}
	}

	@Override
	public void onStop(double time_stamp) {
		// TODO Auto-generated method stub

	}

	public void setMotionMagic(double magicSetpoint, int cruiseVelocity, double secsToMaxSpeed){
		this.magicSetpoint = magicSetpoint; 
		this.cruiseVelocity = cruiseVelocity; 
		this.secsToMaxSpeed = secsToMaxSpeed; 
	}

	public void setOpenLoopSpeed(double openLoopSpeed){
		this.openLoopSpeed = openLoopSpeed; 
	}

	public void setElevatorstate(ElevatorControlState state) {
		mControlState = state;
	}

	public ElevatorControlState getControlState() {
		return mControlState;
	}

}