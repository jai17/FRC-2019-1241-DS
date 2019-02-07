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

public class TankDrive extends Command {
  DrivetrainLoop driveLoop;
  Vision vision; 
  Drivetrain drive; 

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
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

      xVal = vision.avg();
      degreesToTarget = vision.pixelToDegree(xVal); 

      if (Robot.m_oi.getDriveStartButton()){
        drive.turnDrive(drive.getAngle() - degreesToTarget, 0.5, 1);
      }

    if (Robot.m_oi.getDriveRightBumper()) {
      driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
		  driveLoop.setLeftDrive(-0.25 * Robot.m_oi.getDriveLeftY());
			driveLoop.setRightDrive(0.25 * Robot.m_oi.getDriveRightY());
		}else if (Robot.m_oi.getDriveRightTrigger()) {
      driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
			driveLoop.setLeftDrive(-0.25 * Robot.m_oi.getDriveLeftY());
			driveLoop.setRightDrive(0.25 * Robot.m_oi.getDriveRightY());
    } else {
      driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
      driveLoop.setLeftDrive(-0.25 * Robot.m_oi.getDriveLeftY());
			driveLoop.setRightDrive(0.25 * Robot.m_oi.getDriveRightY());
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
