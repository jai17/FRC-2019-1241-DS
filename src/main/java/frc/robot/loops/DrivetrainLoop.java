package frc.robot.loops;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Point;

public class 	DrivetrainLoop implements Loop {

	private static DrivetrainLoop mInstance;
	private Drivetrain drive;

	// Open loop constants
	private double openLoopLeft = 0;
	private double openLoopRight = 0;
	private boolean wantLow = true;

	// Point following constants
	private Point goal = new Point();
	private double topSpeed = 0;

	// Lock constants
	private double lockPos = 0;
	private double lockAng = 0;

	// PID contants
	private boolean drivePID = true; 
	private double distance;
	private double speed;
	private double tolerance;
	private double angle;
	private double stick;

	public enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		POINT_FOLLOWING, // used for autonomous driving
		VISION_TRACKING, // tracking with vision
		LOCK,  // lock drive when scoring
		PID //PIDSetpoint
	}

	private DriveControlState mControlState = DriveControlState.PID;

	public static DrivetrainLoop getInstance() {
		if (mInstance == null) {
			mInstance = new DrivetrainLoop();
			return mInstance;
		} else
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
			if (wantLow) {
				drive.shiftLow();
			} else {
				drive.shiftHigh();
			}
			drive.runLeftDrive(openLoopLeft);
			drive.runRightDrive(openLoopRight);
			return;
		case POINT_FOLLOWING:
			// add point following code here
			return;
		case VISION_TRACKING:
			//drive.regulatedDrivePID(distance, angle, tolerance, 1, 0.5);
			//drive.drivePID(distance, angle, speed, tolerance);
			// drive.changeGyroGains(0.5, 0, 0.01);`
			if (wantLow) {
				drive.shiftLow();
			} else {
				drive.shiftHigh();
			}
			drive.trackTurnPID(angle, speed, tolerance, stick);
			return;
		case LOCK:
			//drive.drivePID(lockPos, lockAng, speed, 0);
			return;
		case PID:
			if (drivePID){
				drive.drivePID(distance, angle, speed, tolerance);
			} else {
				drive.turnPID(angle, speed, tolerance);
			}

			if (wantLow) {
				drive.shiftLow();
			} else {
				drive.shiftHigh();
			}
			return;
		}
	}

	@Override
	public void onStop(double time_stamp) {
		// TODO Auto-generated method stub

	}

	// set control state of drive
	public void setDriveState(DriveControlState state) {
		mControlState = state;
	}

	// get the control state of the drive
	public DriveControlState getControlState() {
		return mControlState;
	}

	// OPEN LOOP
	// set value for left side to drive at for open loop
	public void setLeftDrive(double val) {
		this.openLoopLeft = val;
	}

	// set value for left side to drive at for open loop
	public void setRightDrive(double val) {
		this.openLoopRight = val;
	}

	public void setPIDType(boolean PIDType){
		drivePID = PIDType;
	}

	public void setDistancePID(double distance) {
		this.distance = distance;
	}

	public void setSpeedPID(double speed) {
		this.speed = speed;
	}

	public void setTolerancePID(double tolerance) {
		this.tolerance = tolerance;
	}

	public void setAnglePID(double angle){
		this.angle = angle; 
	}

	public void setStick(double stick) {
		this.stick = stick;
	}

	//set the gear desired for the drive (true is high, false is low) 
	public void selectGear(boolean gear) {
		if (!gear)
			wantLow = false;
		else
			wantLow = true;
	}

	// POINT FOLLOWING
	// set goal point
	public void setGoalPoint(Point goal) {
		this.goal = goal;
	}

	// set top speed
	public void setTopSpeed(double topSpeed) {
		this.topSpeed = topSpeed;
	}

	// VISION TRACKING

	// add methods for drivetrain vision state

	// LOCK
	// lock drive with PID
	public void lockDrive() {
		lockPos = drive.getAveragePos();
		lockAng = drive.getAngle();
	}
}