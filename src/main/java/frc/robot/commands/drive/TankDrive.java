/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.ToggleBoolean;

public class TankDrive extends Command {
  DrivetrainLoop driveLoop;
  Vision vision; 
  Drivetrain drive; 

  ToggleBoolean toggle; 

  //For Vision
  double xVal, degreesToTarget; 

  public TankDrive() {
    driveLoop = DrivetrainLoop.getInstance(); 
    vision = Vision.getInstance(); 
    drive = Drivetrain.getInstance();
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    toggle = new ToggleBoolean(); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

      xVal = vision.avg();
      degreesToTarget = vision.pixelToDegree(xVal); 

      if (Robot.m_oi.getDriveStartButton()){
        //drive.regulatedTurnPID(drive.getAngle() - degreesToTarget, 1, 1, 0.5, true);
        drive.turnPID(drive.getAngle() - degreesToTarget, 1, 2);
      }

    //shift gears
    toggle.set(Robot.m_oi.getDriveRightTrigger());
    if (toggle.get()) { //shift low
      driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
      driveLoop.selectGear(true);
    } else if (!toggle.get()) { //shift high
      driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
      driveLoop.selectGear(false);
    }

    //drive motors
    if (Robot.m_oi.getDriveRightBumper()) { //half speed
      driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
		  driveLoop.setLeftDrive(-0.5 * Robot.m_oi.getDriveLeftY());
      driveLoop.setRightDrive(0.5 * Robot.m_oi.getDriveRightY());
      
		} else { //regular
      driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
      driveLoop.setLeftDrive(Robot.m_oi.getDriveLeftY());
			driveLoop.setRightDrive(Robot.m_oi.getDriveRightY());
      }

    //vision tracking
    if (Robot.m_oi.getDriveLeftTrigger()) {
      driveLoop.setDriveState(DriveControlState.VISION_TRACKING);

      //add tracking code here

    }

    //scoring
    if (Robot.m_oi.getDriveAButton()) { //lock drive
      driveLoop.lockDrive();
      driveLoop.setDriveState(DriveControlState.LOCK);
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
}
