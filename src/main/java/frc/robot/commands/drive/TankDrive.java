  /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionTrackingState;
import frc.robot.util.ToggleBoolean;

public class TankDrive extends Command {
  DrivetrainLoop driveLoop;
  Vision vision;
  Drivetrain drive;

  ToggleBoolean toggle;

  // For Vision
  double xVal, degreesToTarget;
  double yVal, degreesToTargetY;

  public TankDrive() {
    driveLoop = DrivetrainLoop.getInstance();
    vision = Vision.getInstance();
    drive = Drivetrain.getInstance();
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    toggle = new ToggleBoolean(0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (!DriverStation.getInstance().isAutonomous()) {

      xVal = vision.avg();
      degreesToTarget = vision.pixelToDegree(xVal);

      yVal = vision.avgY();
      degreesToTargetY = vision.pixelToDegreeY(yVal);
      SmartDashboard.putNumber("Degrees to Target Y", degreesToTargetY);
      SmartDashboard.putNumber("Camera Angle ", vision.getCameraAngle(degreesToTargetY, 42, 28.5, 77));
      SmartDashboard.putNumber("Camera Distance", vision.getDistance(160, Math.toRadians(31.81), vision.getWidth()));
      // SmartDashboard.putNumber("Camera Distance", vision.getDistance(a1, a2, h1,
      // h2));

      if (Robot.m_oi.getDriveLeftBumper()) { // tracking
        driveLoop.setDriveState(DriveControlState.VISION_TRACKING);
        driveLoop.setPIDType(false);
        driveLoop.selectGear(true);
        driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
        System.out.println("Angle To Target: " + (drive.getAngle() - degreesToTarget) + "Current Angle: " + drive.getAngle());
        driveLoop.setSpeedPID(1);
        if (vision.mTrackingState == VisionTrackingState.CARGO_SHIP) {
          driveLoop.setMaxOutput(0.4);
        } else {
          driveLoop.setMaxOutput(0.5);
        }
        driveLoop.setStick(Robot.m_oi.getDriveRightY() * 0.5);
        driveLoop.setTolerancePID(1);

      } else if (Robot.m_oi.getDriveRightBumper()) { // half speed
        driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
        driveLoop.setLeftDrive(-0.5 * Robot.m_oi.getDriveLeftY());
        driveLoop.setRightDrive(0.5 * Robot.m_oi.getDriveRightY());

      } else if ((Math.abs(Robot.m_oi.getDriveRightY()) > 0.05) || (Math.abs(Robot.m_oi.getDriveLeftY()) > 0.05)) { // regular
        driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
        driveLoop.setLeftDrive(-Robot.m_oi.getDriveLeftY());
        driveLoop.setRightDrive(Robot.m_oi.getDriveRightY());

      } else { // no drive
        driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
        driveLoop.setRightDrive(0);
        driveLoop.setLeftDrive(0);
      }
      driveLoop.selectGear(!  Robot.m_oi.getDriveRightTrigger());

    }

    // shift gears

    // //drive motors
    // if (Robot.m_oi.getDriveRightBumper()) { //half speed
    // driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
    // driveLoop.setLeftDrive(-0.5 * Robot.m_oi.getDriveLeftY());
    // driveLoop.setRightDrive(0.5 * Robot.m_oi.getDriveRightY());

    // } else if ((Math.abs(Robot.m_oi.getDriveRightY()) > 0.1) ||
    // (Math.abs(Robot.m_oi.getDriveLeftY()) > 0.1)) { //regular
    // driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
    // driveLoop.setLeftDrive(-Robot.m_oi.getDriveLeftY());
    // driveLoop.setRightDrive(Robot.m_oi.getDriveRightY());
    // }
    // // else {
    // // driveLoop.setLeftDrive(0);
    // // driveLoop.setRightDrive(0);
    // // }

    // //vision tracking
    // if (Robot.m_oi.getDriveLeftTrigger()) {
    // driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
    // driveLoop.setSpeedPID(1);
    // driveLoop.setTolerancePID(1);
    // driveLoop.setDriveState(DriveControlState.VISION_TRACKING);
    // //add tracking code here

    // }

    // //scoring
    // if (Robot.m_oi.getDriveAButton()) { //lock drive
    // driveLoop.lockDrive();
    // driveLoop.setDriveState(DriveControlState.LOCK);
    // }

    // //driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
