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

  boolean toggle = false; 
  String state; 

  boolean openLoop = false; 
  
  public ElevatorCommand() {
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    elevator = Elevator.getInstance(); 
    elevatorLoop = ElevatorLoop.getInstance();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

      if (Robot.m_oi.getToolAButton()){
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_REST_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
        openLoop = false; 
      }
      else if (Robot.m_oi.getToolRightY() > 0.5){
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.39);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
        openLoop = false; 
      }
      else if (Robot.m_oi.getToolRightX()>0.5){
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_MID_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.39);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
        openLoop = false; 
      } 
      else if (Robot.m_oi.getToolRightY() < -0.5){
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_HIGH_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.39);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
        openLoop = false; 
      } 
      // else if (Robot.m_oi.getToolDPadDown()){
      //   elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_HATCH_FEEDER, NumberConstants.ELEVATOR_MAX_SPEED, 0.39);
      //   elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
      //   openLoop = false; 
      // } 

      if ((Robot.m_oi.getToolStartButton() || Robot.m_oi.getToolBackButton() && !openLoop)){
        openLoop = true; 
      }

      if (openLoop){
        if (Robot.m_oi.getToolStartButton()){
          elevatorLoop.setOpenLoopSpeed(0.5);
          elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);
        } 
        else if (Robot.m_oi.getToolBackButton()){
          elevatorLoop.setOpenLoopSpeed(-0.5);
          elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);
        } else {
          elevatorLoop.setOpenLoopSpeed(0);
          elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);
        }
      }
      
    }
/*
    if (elevator.getAtBottom()){
      elevator.resetEncoders();
    }*/

    // if (Robot.m_oi.getToolStartButton()){
    //   elevatorLoop.setOpenLoopSpeed(1);
    //   elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);
    //   System.out.println("START START START");
    // } else if (Robot.m_oi.getToolBackButton()){
    //   elevatorLoop.setOpenLoopSpeed(-1);
    //   elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);
    //   System.out.println("BACK BACK BACK");
    // } else {
    //   elevatorLoop.setOpenLoopSpeed(0);
    //   elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);
    //   System.out.println("NOTHING NOTHING NOTHING");
    // }



  //}

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
