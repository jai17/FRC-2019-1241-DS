/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.NumberConstants;
import frc.robot.Robot;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionTrackingState;
import frc.robot.util.FieldPoints;
import frc.robot.util.Logger;

public class TrackDrive extends Command {
  private boolean eject; //whether we are ejecting or not
                         //if false, feeding; if true, ejecting

  private double endDist; //distance to stop at
  private DrivetrainLoop driveLoop;
  private Drivetrain drive;
  private Vision vision;
  private double degreesToTarget;
  private double distance;

  private Timer rangeTimer; //timer for scoring
  private boolean rangeTimerStarted; //whether timer has started or not
  private boolean isFinished; //whether the command is finished or not

  private Logger logger;

  public TrackDrive(boolean eject) {
    requires(Robot.drive);

    this.eject = eject;

    //change end condition distances
    if (eject) { 
      this.endDist = FieldPoints.ROCKET_EJECT_DIST;
    } else {
      this.endDist = FieldPoints.FEEDER_EJECT_DIST;
    }

    driveLoop = DrivetrainLoop.getInstance();
    vision = Vision.getInstance();
    drive = Robot.drive;
    logger = Logger.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveLoop.setDriveState(DriveControlState.VISION_TRACKING); //vision
    driveLoop.setPIDType(false); //turning
    driveLoop.setSpeedPID(1); //speed
    driveLoop.setTolerancePID(1); //tolerance

    rangeTimer = new Timer();
    rangeTimerStarted = false;
    isFinished = false;

    updateAngle();
    new SetClawCommand(true).start();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    distance = Robot.drive.getRangeDist();
    updateAngle();

    if (eject) {
      ejectOutput();
    } else {
      feedOutput();
      logger.logd("TrackDrive: ", "updating");
    }
  } 

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //first time true, start loop
    if (distance < endDist && !rangeTimerStarted) { 
      rangeTimer.start();
      rangeTimerStarted = true;
    }

    //if the timer has started
    if (rangeTimerStarted) { 
      if (distance < endDist) { //if within distance
        if (rangeTimer.get() > 0.25) { //and waited for more than half a second
            isFinished = true;
        }
      } else {
        rangeTimerStarted = false; 
        rangeTimer.reset();
      }
    }

    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (eject) {
      new EjectHatchSequence().start(); //eject
    } else {
      new SetClawCommand(false).start(); //open claws
    }
    logger.logd("TrackDrive: ", "ENDING");
    logger.close();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  private void feedOutput() {
    double output;
    if (Math.abs(degreesToTarget) > 20){
      output = 0;
    } else {
      //output is clamped between a top speed and a minimum speed
      output = Math.min(Math.max(Math.pow(distance, 2) * Math.pow(100, -2), 0.12), 0.5);
    }
    
    driveLoop.setStick(-output);
    logger.logd("TrackDrive", "D"+distance +" O"+output + " A"+degreesToTarget);
  }

  private void ejectOutput() {
    if (Math.abs(degreesToTarget) > 7 && Robot.drive.getRangeDist() > 25) { //too far away, not aligned
      driveLoop.setStick(0);
    } else {
      double distance = Robot.drive.getRangeDist();
      double rampDist = 36;
      double rampDistHi = 40;
      double output = 0;
      //high scoring
      if (Robot.elevator.getElevatorEncoder() > NumberConstants.ELEVATOR_HIGH_HATCH_POSITION - 5) {
        
        if (distance > rampDist) { //ramp in 
          output = Math.max(0.5, 0.5 - Math.min(Math.max(Math.pow(distance, 3) * Math.pow(rampDistHi, -3), 0.1), 0.5));
        } else { //ramp out
          output = Math.min(Math.max(Math.pow(distance, 3) * Math.pow(rampDistHi, -3), 0.1), 0.5);
        }

      //mid scoring
      } else if (Robot.elevator.getElevatorEncoder() > NumberConstants.ELEVATOR_MID_HATCH_POSITION - 5) {
        if (distance > rampDist) {
          output = 0.42;
        } else {
          output = (0.42/rampDist)*distance;
        }

      //low scoring
      } else if (Robot.elevator.getElevatorEncoder() > NumberConstants.ELEVATOR_LOW_HATCH_POSITION - 5) {
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
  }

  private void updateAngle() {
    if (vision.getTrackingState() == VisionTrackingState.NO_TARGET) {
      driveLoop.setAnglePID(drive.getAngle());

    } else {
      degreesToTarget = vision.pixelToDegree(vision.avg());
      driveLoop.setAnglePID(drive.getAngle() - degreesToTarget);
    }
  }
}
