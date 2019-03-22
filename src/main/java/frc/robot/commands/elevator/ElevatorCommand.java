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

      //motion magic controls
      if (Robot.m_oi.getToolAButton()) { //bottom
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_REST_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
        openLoop = false; 

      } else if (Robot.m_oi.getToolRightY() > 0.5) { //low rocket & feeder
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.3);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
        openLoop = false; 

      } else if (Robot.m_oi.getToolRightX( )> 0.5) { //mid rocket
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_MID_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.3);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
        openLoop = false; 

      } else if (Robot.m_oi.getToolRightY() < -0.5) { //high rocket
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_HIGH_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.4);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
        openLoop = false; 

      } else if (Robot.m_oi.getToolYButton()) { //cargo ship
        elevatorLoop.setMotionMagic(NumberConstants.ELEVATOR_CARGOSHIP_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.3);
        elevatorLoop.setElevatorstate(ElevatorControlState.MOTION_MAGIC);
      } //end motion magic

      //change open loop state
      if ((Robot.m_oi.getToolStartButton() || Robot.m_oi.getToolBackButton() && !openLoop)){
        openLoop = true; 
      }

      /**
       * if (Robot.m_oi.getToolLeftTrigger() && !openLoop) {
       *  openLoop = true;
       * }
       * 
       * if (openLoop) {
       *  use joystick for manual up/down, default to 10%
       * }
       */

      //open loop controls
      if (openLoop) {
        if (Robot.m_oi.getToolStartButton()){ //up
          elevatorLoop.setOpenLoopSpeed(0.75);
          elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);

        } else if (Robot.m_oi.getToolBackButton()){ //down
          elevatorLoop.setOpenLoopSpeed(-0.6);
          elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);

        } else {
          elevatorLoop.setOpenLoopSpeed(0.1); //hold position
          elevatorLoop.setElevatorstate(ElevatorControlState.OPEN_LOOP);
        }
      } //end open loop
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
