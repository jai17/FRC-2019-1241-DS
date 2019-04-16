/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.NumberConstants;
import frc.robot.Robot;
import frc.robot.loops.CargoLoop;
import frc.robot.loops.CarriageLoop;
import frc.robot.loops.CargoLoop.CargoControlState;
import frc.robot.loops.CarriageLoop.CarriageControlState;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Carriage;
import frc.robot.util.ToggleBoolean;

public class CargoCommand extends Command {

  CargoLoop cargoLoop;
  Carriage carriage;
  CarriageLoop carriageLoop; 
  Cargo cargo;
  boolean openLoop = false; 

  ToggleBoolean toggle; 

  Timer timer;
  private boolean started = false;

  public CargoCommand() {
    requires(Robot.cargo);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    cargoLoop = CargoLoop.getInstance();
    cargo = Cargo.getInstance();
    carriage = Carriage.getInstance();
    carriageLoop = CarriageLoop.getInstance(); 
    timer = new Timer();
    toggle = new ToggleBoolean(1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //if (!Robot.m_oi.getToolLeftTrigger()) { // As long as Left Trigger is not held

      if (!cargo.getOptic() && started == false) {
        timer.start();
        started = true;
      }
      if (!cargo.getOptic() && timer.get() > 0.005) {
        timer.reset();
        timer.stop();
        cargo.setContains(true);
        cargo.stop();
      }
      if (cargo.getOptic()) {
        started = false;
        timer.reset();
        timer.stop();
        cargo.setContains(false);
      }

      
      // if (carriage.getOptic() && !cargo.isCargoPresent()){
      //   carriageLoop.setFeederSpeed(0);
      //   carriageLoop.setIsFeeding(true);
      //   }
      
      if (Robot.m_oi.getToolLeftBumper()) { //outtake
        cargoLoop.setIsIntaking(false);
        cargoLoop.setRollerSpeed(1);
        cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);

      } else if (Robot.m_oi.getToolRightBumper()) { //intaking

        if (Math.abs(cargo.getCargoAngle()) <= 250 && !cargo.isCargoPresent()) { //pivot up, has ball
          cargoLoop.setIsIntaking(true); //intaking
          cargoLoop.setRollerSpeed(1);
          // carriageLoop.setIsFeeding(true); //feeding
          // carriageLoop.setFeederSpeed(1);

          //carriageLoop.setIsShooting(true); //need isShooting, not shooting
          carriageLoop.setShooterSpeed(0);
          carriageLoop.setCarriageState(CarriageControlState.OPEN_LOOP);

        }  else if (Math.abs(cargo.getCargoAngle()) <= 1500 && !cargo.isCargoPresent()) { //pivot down, has ball
          cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
          // carriageLoop.setIsFeeding(true); //feeding
          // carriageLoop.setFeederSpeed(1);
          // carriageLoop.setIsShooting(true); //need isShooting, not shooting
          carriageLoop.setShooterSpeed(0);
          cargoLoop.setIsIntaking(false); //intaking
          cargoLoop.setRollerSpeed(0);

        } else if (cargo.isCargoPresent() && Math.abs(cargo.getCargoAngle()) > 900) { //pivot down, no ball
          cargoLoop.setIsIntaking(true);
          cargoLoop.setRollerSpeed(1);
          // carriageLoop.setIsFeeding(false);
          // carriageLoop.setFeederSpeed(0);
          cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);

        } else { //button pressed, no ball
          cargoLoop.setRollerSpeed(0);
          // carriageLoop.setIsFeeding(true);
          // carriageLoop.setFeederSpeed(0);
        }
      } else { //no ball
        cargoLoop.setIsIntaking(true);
        cargoLoop.setRollerSpeed(0);
        // carriageLoop.setIsFeeding(false);
        // carriageLoop.setFeederSpeed(0);
        cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
      }

      if (!cargo.isCargoPresent() && Math.abs(cargo.getCargoAngle()) <= 900 && carriage.getOptic()){
          carriageLoop.setIsFeeding(true);
          carriageLoop.setFeederSpeed(1);
      } else if (!Robot.m_oi.getToolBButton() && !Robot.m_oi.getToolDPadDown() && !Robot.m_oi.getToolXButton() && !Robot.m_oi.getToolLeftTrigger()){
        carriageLoop.setFeederSpeed(0);
        carriageLoop.setIsShooting(false); 
      }

      //open loop      
      if ((Robot.m_oi.getToolLeftY() > 0.5 || Robot.m_oi.getToolLeftY() < -0.5 && !openLoop)){
        openLoop = true; 
      }

      //not open loop
      if ((Robot.m_oi.getToolRightTrigger()) && openLoop){
        openLoop = false; 
      }  else if (Robot.m_oi.getToolLeftX() < -0.6 && openLoop) {
        openLoop = false;
      } else if (Robot.m_oi.getToolLeftX() > 0.6 && openLoop) {
        openLoop = false;
      }

      //Magic Motion   
      if(!openLoop) {
        if (!cargo.isCargoPresent()){ //has cargo
          cargoLoop.setMotionMagic(NumberConstants.CARGO_FEEDING_ANGLE, NumberConstants.CARGO_MAX_SPEED, 0.1);
          cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
          openLoop = false; 

        } else if (Robot.m_oi.getToolRightTrigger()) { //no cargo, pivot button pressed
          cargoLoop.setMotionMagic(NumberConstants.CARGO_INTAKING_ANGLE, NumberConstants.CARGO_MAX_SPEED, 0.1);
          cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
          openLoop = false; 

        } else if (cargo.isCargoPresent()) { //has no cargo
          if (Robot.m_oi.getToolLeftX() < -0.6) { //no cargo, left x
            // cargoLoop.setMotionMagic(NumberConstants.CARGO_STATION_ANGLE, NumberConstants.CARGO_MAX_SPEED, 0.1);
            // cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
            // openLoop = false; 
          } else { //no cargo, no x
            cargoLoop.setMotionMagic(NumberConstants.CARGO_RESTING_ANGLE, NumberConstants.CARGO_MAX_SPEED, 0.1);
            cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
            openLoop = false;
          }
        } 
      }

        if (openLoop){
          if (Robot.m_oi.getToolLeftY() > 0.1) { //move down
            cargoLoop.setPivotSpeed(0.75 * -Robot.m_oi.getToolLeftY());
            cargoLoop.setCargoState(CargoControlState.OPEN_LOOP);
          } else if (Robot.m_oi.getToolLeftY() < -0.1) { //move up
            cargoLoop.setPivotSpeed(-Robot.m_oi.getToolLeftY());
            cargoLoop.setCargoState(CargoControlState.OPEN_LOOP);
          } else { //don't move
            cargoLoop.setPivotSpeed(0);
            cargoLoop.setCargoState(CargoControlState.OPEN_LOOP);
          }
        }


      // //feeder wheel 
      // if(Robot.m_oi.getToolXButton()) {
      //   carriageLoop.setFeederSpeed(1);
      //   carriageLoop.setIsFeeding(true);
      // } else {
      //   carriageLoop.setFeederSpeed(0);
      // }
      //For Sequence
    //}



    // if (Robot.m_oi.getToolRightTrigger()){
    //     cargoLoop.setMotionMagic(NumberConstants.CARGO_INTAKING_ANGLE , NumberConstants.CARGO_MAX_SPEED, 0.1);
    //     cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
    //   } else {
    //     cargoLoop.setMotionMagic(NumberConstants.CARGO_FEEDING_ANGLE , NumberConstants.CARGO_MAX_SPEED, 0.1);
    //     cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
    // }



  //}
    //   if (Robot.m_oi.getToolLeftY() > 0.5){
    //     cargoLoop.setPivotSpeed(1);
    //     cargoLoop.setCargoState(CargoControlState.OPEN_LOOP);
    //   } else if (Robot.m_oi.getToolLeftY() < -0.5){
    //     cargoLoop.setPivotSpeed(-1);
    //     cargoLoop.setCargoState(CargoControlState.OPEN_LOOP);
    //   } else {
    //     cargoLoop.setPivotSpeed(0);
    //     cargoLoop.setCargoState(CargoControlState.OPEN_LOOP);
    //   }
    // }

    /**Non Loop Methods */
    // if (Robot.m_oi.getToolLeftY() > 0.5){
    //   cargo.runUp(0.5);
    // } else if (Robot.m_oi.getToolLeftY() < -0.5){
    //   cargo.runDown(0.5); 
    //  } else {
    //    cargo.runUp(0);
    //  }

    /**Working Roller */
    //  if (Robot.m_oi.getToolRightBumper()){
    //    cargoLoop.setIsIntaking(true);
    //     cargoLoop.setRollerSpeed(1);
    //  } else if (Robot.m_oi.getToolLeftBumper()){
    //    cargoLoop.setIsIntaking(false);
    //    cargoLoop.setRollerSpeed(1);
    //  } else {
    //    cargoLoop.setIsIntaking(false);
    //    cargoLoop.setRollerSpeed(0);
    //  }

    }

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