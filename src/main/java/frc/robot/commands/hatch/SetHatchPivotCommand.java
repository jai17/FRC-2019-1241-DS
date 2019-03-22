/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.loops.HatchLoop;
import frc.robot.loops.HatchLoop.HatchControlState;

public class SetHatchPivotCommand extends Command {
  HatchLoop hatchLoop; 

	private double setpoint;
	private int cruiseVelocity;
	private double timeToMax;
	private double timeOut;

	public SetHatchPivotCommand(double setPoint, int cruiseVelocity, double timeToMax, double timeOut) {
		this.setpoint = setPoint;
		this.cruiseVelocity = cruiseVelocity;
		this.timeToMax = timeToMax;
		this.timeOut = timeOut;

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		hatchLoop = HatchLoop.getInstance(); 
	hatchLoop.setHatchState(HatchControlState.MOTION_MAGIC);
	setTimeout(timeOut);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		hatchLoop.setMotionMagic(setpoint, cruiseVelocity, timeToMax);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || (Math.abs(Robot.hatch.getMotionMagicError()) < 50);
	}

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
