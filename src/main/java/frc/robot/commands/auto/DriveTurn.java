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
import frc.robot.subsystems.Vision;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.command.Command;

public class DriveTurn extends Command {

  Drivetrain drive = Drivetrain.getInstance();
  DrivetrainLoop driveLoop = DrivetrainLoop.getInstance();
  Vision vision = Vision.getInstance();
  Logger logger = Logger.getInstance();

  double angle;
  double speed;
  double timeOut;
  double tolerance;
  boolean track = false;

  double xVal;
  double degreesToTarget;
  private double offset;

  public DriveTurn(double angle, double speed, double timeOut, double tolerance) {
    this.angle = angle;
    this.timeOut = timeOut;
    this.speed = speed;
    this.tolerance = tolerance;
    this.track = false;
    requires(drive);
  }

  public DriveTurn(double angle, double speed, double tolerance) {
    this.angle = angle;
    this.timeOut = 0;
    this.speed = speed;
    this.tolerance = tolerance;
    this.track = false;
    requires(drive);
  }

  public DriveTurn(double speed, double tolerance, boolean track) {
    this.speed = speed;
    this.tolerance = tolerance;
    this.track = track;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    offset = 0;
    // drive.reset();
    xVal = vision.avg();
    degreesToTarget = vision.pixelToDegree(xVal) + offset;

    driveLoop.setPIDType(false);
    driveLoop.setTrackPID(track);
    driveLoop.setHighPID(false); 
    if (track) {
      driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
    } else {
      driveLoop.setAnglePID(angle);
    }
    driveLoop.setTolerancePID(tolerance);
    driveLoop.setSpeedPID(speed);
    driveLoop.setDriveState(DriveControlState.PID);

    offset = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (track) {
      xVal = vision.avg();
      degreesToTarget = vision.pixelToDegree(xVal) + offset;
      driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    if (track) {
      if (Math.abs(degreesToTarget) < tolerance) {
        logger.logd("DriveTurn: ", " vision angle within tolerance");
        return true;
      } else {
        return false;
      }
    } else {
      if (Math.signum(angle) == 1) {
        if (Math.abs(angle - drive.getAngle()) < (tolerance)) {
          logger.logd("DriveTurn: ", " + angle within tolerance");
          return true;
        } else {
          return false;
        }
      } else {
        if ((Math.abs((angle - drive.getAngle())) < (tolerance))) {
          logger.logd("DriveTurn: ", " - angle within tolerance");
          return true;
        } else {
          return false;
        }
      }
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drive.runLeftDrive(0);
    drive.runRightDrive(0);
    driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
