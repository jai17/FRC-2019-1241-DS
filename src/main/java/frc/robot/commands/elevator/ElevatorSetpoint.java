package frc.robot.commands.elevator;

import frc.robot.Robot;
import frc.robot.loops.ElevatorLoop;
import frc.robot.loops.ElevatorLoop.ElevatorControlState;
import edu.wpi.first.wpilibj.command.Command;

/**
 */
public class ElevatorSetpoint extends Command {

    ElevatorLoop elevatorLoop; 

	private double setpoint;
	private int cruiseVelocity;
	private double timeToMax;
	private double timeOut;

	public ElevatorSetpoint(double setPoint, int cruiseVelocity, double timeToMax, double timeOut) {
		this.setpoint = setPoint;
		this.cruiseVelocity = cruiseVelocity;
		this.timeToMax = timeToMax;
		this.timeOut = timeOut;

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(timeOut);
		elevatorLoop = ElevatorLoop.getInstance(); 
		elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
		elevatorLoop.setMotionMagic(setpoint, cruiseVelocity, timeToMax);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || (Math.abs(Robot.elevator.getMotionMagicError()) < 200);// Robot.elevator.elevatorPIDDone() || isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}