package frc.robot.loops;

import frc.robot.commands.hatch.HatchFeedSequence;
import frc.robot.subsystems.Hatch;

public class HatchLoop implements Loop {

	private static HatchLoop mInstance;
	private Hatch hatch;

	// Open Loop Constants
	public double rollerSpeed = 0;
	public double pivotSpeed = 0;
	private boolean isIntaking = false;

	// Motion Magic Constants
	public double magicSetpoint = 0;
	public int cruiseVelocity = 0;
	public double secsToMaxSpeed = 0;

	// The roller would be fixed to all control modes
	public enum HatchControlState {
		OPEN_LOOP, // open loop voltage control
		MOTION_MAGIC, // motion magic pivot control
		PID_SETPOINT, // stock pid control
	}

	private HatchControlState mControlState = HatchControlState.OPEN_LOOP;

	public static HatchLoop getInstance() {
		if (mInstance == null) {
			mInstance = new HatchLoop();
			return mInstance;
		} else
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
			hatch.pivotHatch(pivotSpeed);
			hatch.intake(rollerSpeed);
			return;
		case MOTION_MAGIC:
		if (HatchFeedSequence.getInstance().isRunning()){
			hatch.magicMotionSetpoint(magicSetpoint, cruiseVelocity, secsToMaxSpeed);
			hatch.intake(1);
		}
			hatch.magicMotionSetpoint(magicSetpoint, cruiseVelocity, secsToMaxSpeed);
			hatch.intake(rollerSpeed);
			return;
		case PID_SETPOINT:
			hatch.intake(rollerSpeed);
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

	public HatchControlState getControlState() {
		return mControlState;
	}

}