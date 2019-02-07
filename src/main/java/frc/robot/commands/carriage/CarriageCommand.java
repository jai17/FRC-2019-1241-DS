/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.carriage;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.loops.CarriageLoop;
import frc.robot.loops.CarriageLoop.CarriageControlState;
import frc.robot.subsystems.Carriage;

public class CarriageCommand extends Command {
  CarriageLoop carriageLoop;
  Carriage carriage;

  //false is within frame perimeter
  private boolean sliderToggle = false, clawToggle = false, ejectToggle = false;
  private double shooterSpeed = 0;

  public CarriageCommand() {
    carriageLoop = CarriageLoop.getInstance();
    carriage = Carriage.getInstance();
    requires(carriage);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    carriageLoop.setCarriageState(CarriageControlState.OPEN_LOOP);

    if (Robot.m_oi.getToolXButton()) { // claw
      clawToggle = !clawToggle;
    } else if (Robot.m_oi.getToolYButton()) { //slider
      sliderToggle = !sliderToggle;
    } else if (Robot.m_oi.getDriveLeftBumper()) { //ejector
      ejectToggle = !ejectToggle;
    }

    if (Robot.m_oi.getToolAButton()) {
      shooterSpeed = 1.0;
    } else {
      shooterSpeed = 0;
    }

    carriageLoop.setClawPos(clawToggle);
    carriageLoop.setSliderPos(sliderToggle);
    carriageLoop.setEjectorPos(ejectToggle);

    carriageLoop.setIsShooting(true);
    carriageLoop.setShooterSpeed(shooterSpeed);
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
