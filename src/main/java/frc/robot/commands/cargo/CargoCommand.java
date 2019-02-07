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
import frc.robot.loops.CargoLoop.CargoControlState;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Carriage;
import frc.robot.util.ToggleBoolean;

public class CargoCommand extends Command {

  CargoLoop cargoLoop;
  Carriage carriage;
  Cargo cargo;

  ToggleBoolean toggle = new ToggleBoolean();

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
    timer = new Timer();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (!Robot.m_oi.getToolLeftTrigger()) { // As long as Left Trigger is not held

      if (!cargo.getOptic() && started == false) {
        timer.start();
        started = true;
      }
      if (!cargo.getOptic() && timer.get() > 0.25) {
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

      if (Robot.m_oi.getToolLeftBumper()) {
        cargoLoop.setIsIntaking(false);
        cargoLoop.setRollerSpeed(0.8);
        cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
      } else if (Robot.m_oi.getToolRightBumper()) {
        if (cargo.getCargoAngle() >= 60 && cargo.isCargoPresent()) {
          cargoLoop.setIsIntaking(true);
          cargoLoop.setRollerSpeed(0.8);
          cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
          carriage.feederIn(0.75);
        } else if (!cargo.isCargoPresent() && cargo.getCargoAngle() < 60) {
          cargoLoop.setIsIntaking(true);
          cargoLoop.setRollerSpeed(0.8);
          cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
        } else {
          cargoLoop.setRollerSpeed(0);
        }
      } else {
        cargoLoop.setPivotSpeed(0);
        cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
      }

      toggle.set(Robot.m_oi.getToolRightTrigger());

      if (Robot.m_oi.getToolRightTrigger()) {
        if (toggle.get()) {
          cargoLoop.setMotionMagic(NumberConstants.CARGO_INTAKING_ANGLE, NumberConstants.CARGO_MAX_SPEED, 1);
          cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
        } else {
          cargoLoop.setMotionMagic(NumberConstants.CARGO_FEEDING_ANGLE, NumberConstants.CARGO_MAX_SPEED, 1);
          cargoLoop.setCargoState(CargoControlState.MOTION_MAGIC);
        }
      }

      if (Robot.m_oi.getToolLeftY() > 0.5){
        cargoLoop.setPivotSpeed(0.8);
        cargoLoop.setCargoState(CargoControlState.OPEN_LOOP);
      } else if (Robot.m_oi.getToolLeftY() < -0.5){
        cargoLoop.setPivotSpeed(-0.8);
        cargoLoop.setCargoState(CargoControlState.OPEN_LOOP);
      }

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
