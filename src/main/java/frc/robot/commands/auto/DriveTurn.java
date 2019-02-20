/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain; 

import edu.wpi.first.wpilibj.command.Command;


public class DriveTurn extends Command {

  Drivetrain drive = Drivetrain.getInstance(); 
  DrivetrainLoop driveLoop = DrivetrainLoop.getInstance(); 

  double angle; 
  double speed;
  double timeOut; 
  double tolerance; 

  public DriveTurn( double angle,double speed,double timeOut,double tolerance) {
    this.angle = angle; 
    this.timeOut = timeOut; 
    this.speed = speed; 
    this.tolerance = tolerance; 
    requires(drive);
  }

  public DriveTurn( double angle,double speed,double tolerance) {
    this.angle = angle; 
    this.timeOut = 0; 
    this.speed = speed; 
    this.tolerance = tolerance; 
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drive.resetEncoders();

    driveLoop.setPIDType(false);
    driveLoop.setAnglePID(angle);
    driveLoop.setTolerancePID(tolerance);
    driveLoop.setSpeedPID(speed);
    driveLoop.setDriveState(DriveControlState.PID);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ((Math.abs(drive.getAngle()) - Math.abs(angle)) > Math.abs(angle)) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
    System.out.println(this.toString() + " HAS FINISHED");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
