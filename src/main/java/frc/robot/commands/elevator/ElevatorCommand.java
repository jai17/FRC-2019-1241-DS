/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.NumberConstants;
import frc.robot.Robot;
import frc.robot.loops.ElevatorLoop;
import frc.robot.loops.ElevatorLoop.ElevatorControlState;
import frc.robot.subsystems.Elevator;
import frc.robot.util.ToggleBoolean;

public class ElevatorCommand extends Command {
  Elevator elevator; 
  ElevatorLoop elevatorLoop; 

  ToggleBoolean toggle; 
  
  public ElevatorCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    elevator = Elevator.getInstance(); 
    elevatorLoop = ElevatorLoop.getInstance(); 
    toggle = new ToggleBoolean(); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    toggle.set(Robot.m_oi.getDriveYButton());

    if (!toggle.get()){

      if (Robot.m_oi.getToolAButton()){
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_REST_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 1);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
      }
      else if (Robot.m_oi.getToolRightY() < -0.5){
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_LOW_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 1);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
      }
      else if (Robot.m_oi.getToolRightX()>0.5){
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_MID_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 1);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
      } 
      else if (Robot.m_oi.getToolRightY() > 0.5){
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_HIGH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 1);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC); 
      }

    } else {
      if (Robot.m_oi.getDriveStartButton()){
        elevatorLoop.setOpenLoopSpeed(0.25);
      } else if (Robot.m_oi.getDriveBackButton()){
        elevatorLoop.setOpenLoopSpeed(-0.25);
      }
        elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);
    }

    if (elevator.getAtBottom()){
      elevator.resetEncoders();
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
