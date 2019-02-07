package frc.robot.loops;

import frc.robot.subsystems.Cargo;

public class CargoLoop implements Loop {

	private static CargoLoop mInstance;
	private Cargo cargo;

	// Open Loop Constants
	public double rollerSpeed = 0;
	public double pivotSpeed = 0;
	public boolean isIntaking = true;

	// Motion Magic Constants
	public double magicSetpoint;
	public int cruiseVelocity;
	public double secsToMaxSpeed;

	// PID Constants

	// The roller would be fixed to all control modes
	public enum CargoControlState {
		OPEN_LOOP, // open loop voltage control
		MOTION_MAGIC, // motion magic pivot control
		PID_SETPOINT, // stock pid control
	}

	private CargoControlState mControlState = CargoControlState.MOTION_MAGIC;

	public static CargoLoop getInstance() {
		if (mInstance == null) {
			mInstance = new CargoLoop();
			return mInstance;
		} else
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
			if (pivotSpeed > 0) {
				cargo.runUp(Math.abs(pivotSpeed));
			} else {
				cargo.runDown(Math.abs(pivotSpeed));
			}
			cargo.intake(rollerSpeed);
			return;
		case MOTION_MAGIC:
			cargo.magicMotionSetpoint(magicSetpoint, cruiseVelocity, secsToMaxSpeed);
			cargo.intake(rollerSpeed);
			return;
		case PID_SETPOINT:
			cargo.intake(rollerSpeed);
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

	public void setMotionMagic(double magicSetpoint, int cruiseVelocity, double secsToMaxSpeed) {
		this.magicSetpoint = magicSetpoint;
		this.cruiseVelocity = cruiseVelocity;
		this.secsToMaxSpeed = secsToMaxSpeed;
	}

	public void setIsIntaking(boolean intaking) {
		if (intaking) {
			isIntaking = true;
		} else {
			isIntaking = false;
		}
	}

	public void setRollerSpeed(double speed) {
		if (isIntaking) {
			rollerSpeed = speed;
		} else {
			rollerSpeed = -speed;
		}
	}

	public void setPivotSpeed(double speed) {
		pivotSpeed = speed;
	}

	public CargoControlState getControlState() {
		return mControlState;
	}

}