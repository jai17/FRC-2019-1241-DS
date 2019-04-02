/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.routines.LevelTwoSequence;
import frc.robot.loops.CargoLoop;
import frc.robot.loops.CargoLoop.CargoControlState;
import frc.robot.subsystems.Cargo;

public class SetIntakePivotCommand extends Command {
	//   CargoLoop cargoLoop;
	  Cargo cargo;

	private double setpoint;
	private int cruiseVelocity;
	private double timeToMax;
	private double timeOut;

	public SetIntakePivotCommand(double setPoint, int cruiseVelocity, double timeToMax, double timeOut) {
		this.setpoint = setPoint;
		this.cruiseVelocity = cruiseVelocity;
		this.timeToMax = timeToMax;
		this.timeOut = timeOut;
		cargo = Cargo.getInstance();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(timeOut);
		// cargoLoop = CargoLoop.getInstance(); 
    	// cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		cargo.magicMotionSetpoint(setpoint, cruiseVelocity, timeToMax);
		System.out.println(this.toString() + " is being naughty");
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || Math.abs(Robot.cargo.getMotionMagicError()) < 100 ;// Robot.elevator.elevatorPIDDone() || isTimedOut();
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
