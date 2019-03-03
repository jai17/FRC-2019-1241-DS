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
import frc.robot.util.ToggleBoolean;

public class CarriageCommand extends Command {
  CarriageLoop carriageLoop;
  Carriage carriage;

  //false is within frame perimeter
  private ToggleBoolean sliderToggle, clawToggle, ejectToggle;
  private double shooterSpeed = 0;

  boolean claw = false; 

  public CarriageCommand() {
    sliderToggle = new ToggleBoolean(0.5);
    clawToggle = new ToggleBoolean(0.5);
    ejectToggle = new ToggleBoolean(0.5);

    carriageLoop = CarriageLoop.getInstance();
    carriage = Carriage.getInstance();
    requires(carriage);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    clawToggle.set(true);
    sliderToggle.set(true);
    ejectToggle.set(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //pneumatic toggles
    if(Robot.m_oi.getToolDPadRight()){ //toggle claw 
      clawToggle.set(Robot.m_oi.getToolDPadRight());
    }
    if(Robot.m_oi.getToolDPadLeft()){ //toggle tray
      sliderToggle.set(Robot.m_oi.getToolDPadLeft());
    }
    if(Robot.m_oi.getToolDPadUp() && carriage.getUltrasonicLeft() <= 5 && carriage.getUltrasonicRight() <=5){ //shoot hatch
      ejectToggle.set(Robot.m_oi.getToolDPadUp());
      clawToggle.set(false);
    }
    if(Robot.m_oi.getToolDPadDown()) { //retract everything
      clawToggle.set(false);
      ejectToggle.set(false);
      sliderToggle.set(false);
    }

    // if(Robot.m_oi.getToolDPadRight()){
    //   claw = !claw; 
    // }

    //shooting
    if (Robot.m_oi.getToolBButton()) { //rocket shot
      shooterSpeed = Robot.shooterSpeed;
      carriageLoop.setIsShooting(true);
    } else if (Robot.m_oi.getToolXButton()) { //cargo shot
      shooterSpeed = 0.4039; //power move
      carriageLoop.setIsShooting(true);
    } else { //no  shot
      shooterSpeed = 0;
      carriageLoop.setIsShooting(false);
    }

    //set pneumatics
    carriageLoop.setClawPos(clawToggle.get());
    carriageLoop.setSliderPos(sliderToggle.get());
    carriageLoop.setEjectorPos(ejectToggle.get());

    //set shooter
    carriageLoop.setShooterSpeed(shooterSpeed);
    carriageLoop.setCarriageState(CarriageControlState.OPEN_LOOP);
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