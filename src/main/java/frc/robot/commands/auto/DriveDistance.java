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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;


public class DriveDistance extends Command {

  Drivetrain drive = Drivetrain.getInstance(); 
  DrivetrainLoop driveLoop = DrivetrainLoop.getInstance(); 
  Vision vision = Vision.getInstance();

  double distance; 
  double angle; 
  double speed;
  double timeOut; 
  double tolerance; 
  boolean track;
  double xVal;
  double degreesToTarget;

  Timer timer; 
  boolean started = true; 

  public DriveDistance(double distance, double angle, double timeOut,  double speed, double tolerance) {
    this.distance = distance; 
    this.angle = angle; 
    this.timeOut = timeOut; 
    this.speed = speed; 
    this.tolerance = tolerance; 
    this.track = false;
    requires(drive);
  }

  //using this one
  public DriveDistance(double distance, double angle, double speed, double tolerance) {
    this.distance = distance; 
    this.angle = angle; 
    this.timeOut = 0; 
    this.speed = speed; 
    this.tolerance = tolerance; 
    this.track = false;

    requires(drive);
  }

  public DriveDistance(double distance, double angle, double timeOut,  double speed, double tolerance, boolean track) {
    this.distance = distance; 
    this.angle = angle; 
    this.timeOut = timeOut; 
    this.speed = speed; 
    this.tolerance = tolerance;
    this.track = track; 
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   timer = new Timer(); 

    drive.reset();

    driveLoop.setPIDType(true);
    driveLoop.setDistancePID(distance);
    driveLoop.setAnglePID(angle);
    driveLoop.setTolerancePID(tolerance);
    driveLoop.setSpeedPID(speed);
    driveLoop.selectGear(false);
    driveLoop.setDriveState(DriveControlState.PID);
    drive.setRightrampRate(0);
    drive.setLeftrampRate(0);

    xVal = vision.avg();
    degreesToTarget = vision.pixelToDegree(xVal);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(track) {
     driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
    }

    driveLoop.selectGear(false);

    if(Math.abs(drive.getAveragePos()) < 50){
      drive.setLeftrampRate(1);
      drive.setRightrampRate(1);
    } else {
      drive.setLeftrampRate(0);
      drive.setRightrampRate(0);
    }

    if(Math.abs(drive.getAveragePos()) >= (Math.abs(distance) - tolerance) && started == false) {
      timer.start(); 
      started = true; 
    } else {
      timer.reset(); 
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
  
    if (started == true && timer.get() > 0.75){
      return true; 
    }
    else {
      started = false; 
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveLoop.setLeftDrive(0);
    driveLoop.setRightDrive(0);
    driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
    System.out.println(this.toString() + " HAS FINISHED");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
