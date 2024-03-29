package frc.robot.loops;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.commands.auto.routines.LevelTwoSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Point;

public class DrivetrainLoop implements Loop {

	private static DrivetrainLoop mInstance;
	private Drivetrain drive;

	// Open loop constants
	private double openLoopLeft = 0;
	private double openLoopRight = 0;
	private boolean wantLow = false;
	private boolean liftEngaged = false;

	// Point following constants
	private Point goal = new Point();
	private double topSpeed = 0;
	private boolean isTurning = false; 

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
	double maxOutput = 0; 
	boolean track = false; 
	boolean highPID = false; 

	//Point Following
	private double[] xy;

	public enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		POINT_FOLLOWING, // used for autonomous driving
		VISION_TRACKING, // tracking with vision
		LOCK,  // lock drive when scoring
		PID //PIDSetpoint
	}

	private DriveControlState mControlState = DriveControlState.OPEN_LOOP;

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
			if (!DriverStation.getInstance().isAutonomous() && 
					!LevelTwoSequence.getInstance().isRunning()) {
				updateXY();

				if (!wantLow) {
					drive.shiftLow();
				} else {
					drive.shiftHigh();
				}

				if (liftEngaged) {
					drive.extendLifter();
				} else {
					drive.retractLifter();
				}

				drive.runLeftDrive(openLoopLeft);
				drive.runRightDrive(openLoopRight);
			} else {
				// drive.runLeftDrive(0);
				// drive.runRightDrive(0);	
			}
			
			return;
		case POINT_FOLLOWING:
			updateXY();
			
			if (drivePID){
				drive.regulatedDrivePID(distance, angle, tolerance, topSpeed, false, highPID);
			} else {
				drive.regulatedTurnPID(angle, tolerance, topSpeed, false);
			}

			// add point following code here
			return;

		case VISION_TRACKING:
			updateXY();

			drive.shiftLow();
			drive.trackTurnPID(angle, speed, tolerance, stick, maxOutput);
			return;
			
		case LOCK:
			//drive.drivePID(lockPos, lockAng, speed, 0);
			return;
		case PID:
			updateXY();

			if (drivePID){
				drive.regulatedDrivePID(distance, angle, tolerance, topSpeed, track, highPID);
			} else {
				drive.regulatedTurnPID(angle, tolerance, speed, track);
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

	public void setTrackPID(boolean track) {
		this.track = track;
	}

	public void setAnglePID(double angle){
		this.angle = angle; 
	}

	public void setStick(double stick) {
		this.stick = stick;
	}
	public void setMaxOutput(double maxOutput) {
		this.maxOutput = maxOutput;
	}

	//set the gear desired for the drive (true is high, false is low) 
	public void selectGear(boolean gear) {
		if (!gear)
			wantLow = false;
		else
			wantLow = true;
	}

	//set the lifter position
	public void engageLifter(boolean lift) {
		if (lift)
			liftEngaged = true;
		else
			liftEngaged = false;
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

	//update xy
	public void updateXY() {
		xy = drive.getXY();
	}
	public boolean getGear(){
		return wantLow; 
	}

	public void setHighPID(boolean highPID){
		this.highPID = highPID; 
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