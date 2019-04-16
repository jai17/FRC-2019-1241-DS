/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveDistance extends Command {

  Drivetrain drive = Drivetrain.getInstance();
  DrivetrainLoop driveLoop = DrivetrainLoop.getInstance();
  Vision vision = Vision.getInstance();
  Carriage carriage = Carriage.getInstance();
  Logger logger = Logger.getInstance(); 

  double distance;
  double angle;
  double speed;
  double timeOut;
  double tolerance;
  boolean track;
  double xVal;
  double degreesToTarget;
  double initDistance = 0;
  boolean relative;
  boolean highPID;
  int stopDrive = 1; 

  Timer timer;
  Timer rangeTimer;
  boolean rangeTimerStarted = false;
  boolean started = true;
  boolean isFinished = false;

  public DriveDistance(double distance, double angle, double timeOut, double speed, double tolerance) {
    this.distance = distance;
    this.angle = angle;
    this.timeOut = timeOut;
    this.speed = speed;
    this.tolerance = tolerance;
    this.track = false;
    this.relative = true;
    this.highPID = false;
    this.stopDrive = 1; 
    requires(drive);
  }

  // using this one
  public DriveDistance(double distance, double angle, double speed, double tolerance) {
    this.distance = distance;
    this.angle = angle;
    this.timeOut = 0;
    this.speed = speed;
    this.tolerance = tolerance;
    this.track = false;
    this.relative = false;
    this.highPID = false;
    this.stopDrive = 1; 
    requires(drive);
  }

  public DriveDistance(double distance, double angle, double speed, double tolerance, int stopDrive, boolean highPID) {
    this.distance = distance;
    this.angle = angle;
    this.timeOut = 0;
    this.speed = speed;
    this.tolerance = tolerance;
    this.stopDrive = stopDrive; 
    this.track = false;
    this.relative = false;
    this.highPID = highPID;

    requires(drive);
  }

  public DriveDistance(double distance, double angle, double speed, double tolerance, boolean track) {
    this.distance = distance;
    this.angle = angle;
    this.speed = speed;
    this.tolerance = tolerance;
    this.track = track;
    this.highPID = false;
    this.stopDrive = 1; 
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer = new Timer();
    rangeTimer = new Timer();

    rangeTimerStarted = false;
    isFinished = false;

    // drive.reset();

    xVal = vision.avg();
    degreesToTarget = vision.pixelToDegree(xVal) - 2.75;

    driveLoop.setPIDType(true);
    driveLoop.setTrackPID(track);
    driveLoop.setHighPID(highPID);
    driveLoop.setDistancePID(distance + drive.getAveragePos() + Math.signum(distance) * tolerance);
    if (track) {
      driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
      setTimeout(3);
    } else {
      driveLoop.setAnglePID(angle);
      setTimeout(1);
    }
    driveLoop.setTolerancePID(tolerance);
    // driveLoop.setSpeedPID(speed);
    driveLoop.setTopSpeed(speed);
    // driveLoop.selectGear(true);
    driveLoop.setDriveState(DriveControlState.PID);
    drive.setRightrampRate(0);
    drive.setLeftrampRate(0);

    initDistance = drive.getAveragePos();
    isFinished = false;
    rangeTimerStarted = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    xVal = vision.avg();
    degreesToTarget = vision.pixelToDegree(xVal)  - 2.75;

    if (track) {
      driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
      // System.out.println(
      // "Distance: " + (drive.getAveragePos() - distance) + " Angle: " +
      // (drive.getAngle() - degreesToTarget));
    }

    // driveLoop.selectGear(false);

    // if ((Math.abs(drive.getAveragePos()) - Math.abs(initDistance)) < 50) {
    // drive.setLeftrampRate(0.5);
    // drive.setRightrampRate(0.5);
    // } else {
    // drive.setLeftrampRate(0);
    // drive.setRightrampRate(0);
    // }

    // if ((Math.abs(drive.getAveragePos()) + distance) >= (Math.abs(distance) -
    // tolerance) && started == false) {
    // timer.start();
    // started = true;
    // } else {
    // timer.reset();
    // started = false;
    // }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    if (track) {
      if (carriage.getUltrasonicLeft() < tolerance && !rangeTimerStarted) {
        rangeTimer.start();
        rangeTimerStarted = true;
      }
      if (rangeTimerStarted) { 
        if (carriage.getUltrasonicLeft() < tolerance){
        if (rangeTimer.get() > 0.5) {
          isFinished = true;
          }
      } else {
        rangeTimerStarted = false; 
        rangeTimer.reset();
      }
      }
      return isFinished;
    } else {

      if (Math.signum(distance) == 1){
        if (drive.getAveragePos() > (initDistance + distance - tolerance)){
          return true; 
        } else {
          return false; 
        }
      } else {
        if (drive.getAveragePos() < (initDistance + distance + tolerance)){
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
    if (stopDrive == 1){
    drive.runLeftDrive(0);
    drive.runRightDrive(0);
    }
    driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
