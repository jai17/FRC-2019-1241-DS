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
import frc.robot.NumberConstants;
import frc.robot.Robot;
import frc.robot.commands.auto.routines.LevelTwoSequence;
import frc.robot.commands.hatch.HatchFeedSequence;
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
  private double offset;

  // For Vision
  double xVal, degreesToTarget;
  double yVal, degreesToTargetY;

  // hatch pick up test
  HatchFeedSequence hatchPickup = HatchFeedSequence.getInstance();
  boolean pickingUp = false;

  // dingus
  boolean lifting = false;

  LevelTwoSequence levelTwoSequence;

  public TankDrive() {
    driveLoop = DrivetrainLoop.getInstance();
    vision = Vision.getInstance();
    drive = Drivetrain.getInstance();
    requires(drive);

    this.offset = 0.5;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    levelTwoSequence = LevelTwoSequence.getInstance();
    toggle = new ToggleBoolean(0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (!DriverStation.getInstance().isAutonomous()) {

      xVal = vision.avg();
      if (vision.getTrackingState() == VisionTrackingState.NO_TARGET) {
        degreesToTarget = 0;
      } else {
        degreesToTarget = vision.pixelToDegree(xVal) + offset;
      }
      // added offset to compensate for camera

      yVal = vision.avgY();
      degreesToTargetY = vision.pixelToDegreeY(yVal);

      if (Robot.m_oi.getDriveRightTrigger()) { //AUTO HATCH FEED
        //change states
        if (vision.mTrackingState == VisionTrackingState.ROCKET
            || vision.mTrackingState == VisionTrackingState.CARGO_SHIP) {
          driveLoop.setDriveState(DriveControlState.VISION_TRACKING);
        } else {
          driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
        }
        System.out.println("RT Feed"); 

        driveLoop.setPIDType(false);
        driveLoop.setSpeedPID(1);

        //change speed based on # of targets
        if (vision.mTrackingState == VisionTrackingState.CARGO_SHIP) {
          driveLoop.setMaxOutput(0.3); // 0.6
        } else {
          driveLoop.setMaxOutput(0.75); // 0.7
        }


        if ((vision.getTrackingState() == VisionTrackingState.ROCKET)
            || (vision.getTrackingState() == VisionTrackingState.CARGO_SHIP)) {
          driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);

          if (Math.abs(degreesToTarget) > 20 && Robot.drive.getRangeDist() > 25){
            driveLoop.setStick(0);
          } else {
            double distance = Robot.drive.getRangeDist();
            //output is clamped between a top speed and a minimum speed
            double output = Math.min(Math.max(Math.pow(distance, 2) * Math.pow(64, -2), 0.2), 0.85);
            driveLoop.setStick(-output);
          }

          driveLoop.setTolerancePID(1);

        } else {
          driveLoop.setLeftDrive(-Robot.m_oi.getDriveLeftY());
          driveLoop.setRightDrive(Robot.m_oi.getDriveRightY());
        }

      } else if (Robot.m_oi.getDriveLeftTrigger()) { //AUTO HATCH PANEL SCORE
        if (vision.mTrackingState == VisionTrackingState.ROCKET
            || vision.mTrackingState == VisionTrackingState.CARGO_SHIP) { //if targets are there
          driveLoop.setDriveState(DriveControlState.VISION_TRACKING);
        } else {
          driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
        }
        System.out.println("LT Score"); 

        driveLoop.setPIDType(false);
        driveLoop.setSpeedPID(1);

        if (vision.mTrackingState == VisionTrackingState.CARGO_SHIP) {
          driveLoop.setMaxOutput(0.3); // 0.6 real robot
        } else {
          driveLoop.setMaxOutput(0.75); // 0.7 real robot
        }

        //ramp down based on rangefinder angle
        if ((vision.getTrackingState() == VisionTrackingState.ROCKET)
            || (vision.getTrackingState() == VisionTrackingState.CARGO_SHIP)) { //if targets available
          driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);

          if (Math.abs(degreesToTarget) > 15 && Robot.drive.getRangeDist() < 25) { //too close, not aligned
            driveLoop.setStick(0);
          } else {
            double distance = Robot.drive.getRangeDist();
            double rampDist = 36;
            double rampDistHi = 40;
            double output = 0;
            //high scoring
            if (Robot.elevator.getElevatorEncoder() > NumberConstants.ELEVATOR_HIGH_HATCH_POSITION - 5) {
              
              if (distance > rampDistHi) { //ramp in 
                output = 0.35;
              } else { //ramp out
                output = Math.min(Math.max(Math.pow(distance, 2) * Math.pow(rampDistHi, -2), 0.1), 0.35);
              }

            //mid scoring
            } else if (Robot.elevator.getElevatorEncoder() > NumberConstants.ELEVATOR_MID_HATCH_POSITION - 5) {
              if (distance > rampDist) {
                output = 0.45;
              } else {
                output = (0.45/rampDist)*distance;
              }

            //low scoring
            // } else if (Robot.elevator.getElevatorEncoder() > NumberConstants.ELEVATOR_LOW_HATCH_POSITION - 5) { //add back for real robot
            } else {
              if (distance > rampDist) {
                output = 0.5;
              } else {
                output = (0.5/rampDist)*distance;
              }
              
            }
            //set the power to output
            driveLoop.setStick(-output);
          }
          
          driveLoop.setTolerancePID(1);

        } else {
          driveLoop.setLeftDrive(-Robot.m_oi.getDriveLeftY());
          driveLoop.setRightDrive(Robot.m_oi.getDriveRightY());
        }

      } else if (Robot.m_oi.getDriveLeftBumper()) { //alignment tracking
        if (vision.mTrackingState == VisionTrackingState.ROCKET
            || vision.mTrackingState == VisionTrackingState.CARGO_SHIP) {
          driveLoop.setDriveState(DriveControlState.VISION_TRACKING);
        } else {
          driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
        }
        driveLoop.setPIDType(false);
        driveLoop.setSpeedPID(1);
        if (vision.mTrackingState == VisionTrackingState.CARGO_SHIP) {
          driveLoop.setMaxOutput(0.3); // 0.6
        } else {
          driveLoop.setMaxOutput(0.75); // 0.7
        }
        // driveLoop.selectGear(true);
        if ((vision.getTrackingState() == VisionTrackingState.ROCKET)
            || (vision.getTrackingState() == VisionTrackingState.CARGO_SHIP)) {
          driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
          driveLoop.setStick(Robot.m_oi.getDriveRightY() * 0.50);
        } else {
          driveLoop.setLeftDrive(-Robot.m_oi.getDriveLeftY());
          driveLoop.setRightDrive(Robot.m_oi.getDriveRightY());
        }
        // driveLoop.setStick(Robot.m_oi.getDriveRightY() * 0.75);
        driveLoop.setTolerancePID(1);

      } else if ((Math.abs(Robot.m_oi.getDriveRightY()) > 0.05) || (Math.abs(Robot.m_oi.getDriveLeftY()) > 0.05)) {
        driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
        // driveLoop.setLeftDrive(-Robot.m_oi.getDriveLeftY());
        // driveLoop.setRightDrive(Robot.m_oi.getDriveRightY());
        if (driveLoop.getGear()) { //HIGH GEAR
          driveLoop.setLeftDrive(-0.90 * Robot.m_oi.getDriveLeftY());
          driveLoop.setRightDrive(0.90 * Robot.m_oi.getDriveRightY());
        } else {
          drive.setLeftrampRate(0.05);
          drive.setRightrampRate(0.05);
          driveLoop.setLeftDrive(0.95 * -Robot.m_oi.getDriveLeftY());
          driveLoop.setRightDrive(0.95 * Robot.m_oi.getDriveRightY());
        }
        // no drive
      } else {
        driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
        driveLoop.setRightDrive(0);
        driveLoop.setLeftDrive(0);
      }

      driveLoop.selectGear(Robot.m_oi.getDriveRightBumper());

      if (Robot.m_oi.getDriveYButton()) {
        if (!LevelTwoSequence.getInstance().isRunning() && !lifting) {
          levelTwoSequence.start();
          System.out.println("LEVEL 2 SEQUENCE RUNNING");
          lifting = true;
        }
      } else {
        if (lifting) {
          levelTwoSequence.cancel();
          System.out.println("LEVEL 2 SEQUENCE CANCELLING");
          lifting = false;
        }
      }
      // if (Robot.m_oi.getDriveLeftTrigger()){
      // driveLoop.engageLifter(true);
      // } else {
      // driveLoop.engageLifter(false);
      // }
    }
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

  private double squareMaintainSign(double input) {
    return Math.signum(input) * Math.pow(input, 2);
  }

  private double capMaintainSign(double input, double cap) {
    if (input > 0) { //above zero
      return Math.min(input, Math.abs(cap));
    } else { //below zero
      return Math.max(input, Math.abs(cap));
    }
  }
}
