/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.carriage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.loops.CarriageLoop;
import frc.robot.loops.CarriageLoop.CarriageControlState;
import frc.robot.subsystems.Carriage;
import frc.robot.util.ToggleBoolean;

public class CarriageCommand extends Command {
  CarriageLoop carriageLoop;
  Carriage carriage;

  // false is within frame perimeter
  private ToggleBoolean sliderToggle, clawToggle, ejectToggle;
  private double shooterSpeed = 0;

  boolean claw = false;

  Timer timer;
  Timer timerTray;
  boolean hasStarted = false;
  boolean trayStarted = false;

  double ejectDelay = 0.05;
  double trayDelay = 0.5;

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
    timer = new Timer();
    timerTray = new Timer();
    // clawToggle.set(false);
    // sliderToggle.set(false); // ejectToggle.set(true) ;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!DriverStation.getInstance().isAutonomous()) {
      // pneumatic toggles
      if(Robot.m_oi.getDriveLeftTrigger() && Robot.drive.getRangeDist() < 9 && Robot.vision.pixelToDegree(Robot.vision.avg()) < 4){
        clawToggle.setTime(2);
        ejectToggle.setTime(2);
        sliderToggle.setTime(2);
        clawToggle.set(true);
        ejectToggle.set(false);
        if (!carriageLoop.getSliderPos()){
        sliderToggle.set(true);
        }
        // System.out.println("Ejecting");
      }

      if (Robot.m_oi.getDriveRightTrigger() && Robot.drive.getRangeDist() < 11
          && Robot.vision.pixelToDegree(Robot.vision.avg()) < 4) {
        clawToggle.setTime(2);
        clawToggle.set(false);
        // System.out.println("Feeding");
      } else if (Robot.m_oi.getToolDPadRight()) { // toggle claw
        clawToggle.setTime(0.5);
        clawToggle.set(Robot.m_oi.getToolDPadRight());
      }
      if (Robot.m_oi.getToolDPadLeft()) { // toggle tray
        sliderToggle.set(Robot.m_oi.getToolDPadLeft());
      }

      if (Robot.m_oi.getToolDPadUp()) { // shoot hatch
        clawToggle.set(false);
        if (!hasStarted) {
          timer.start();
          timerTray.start();
        }
        hasStarted = true;
        trayStarted = true;
      }

      if (hasStarted) { // ejector timer
        if (timer.get() > ejectDelay) {
          ejectToggle.setTime(0.5);
          ejectToggle.set(true);
          hasStarted = false;
          timer.reset();
        }
      }

      if (trayStarted) { // tray timer
        if (timerTray.get() > (ejectDelay + trayDelay)) {
          if (sliderToggle.get())
            sliderToggle.set(false);
          trayStarted = false;
        }
      }

      // shooting
      if (Robot.m_oi.getToolBButton()) { // rocket shot
        shooterSpeed = 0.70;
        carriageLoop.setFeederSpeed(1);
        carriageLoop.setIsShooting(true);
      } else if (Robot.m_oi.getToolXButton()) { // cargo shot
        shooterSpeed = 0.4560; // jash
        carriageLoop.setFeederSpeed(1);
        carriageLoop.setIsShooting(true);
      } else if (Robot.m_oi.getToolDPadDown()) { // run feeder
        carriageLoop.setIsShooting(false);
        carriageLoop.setIsFeeding(true);
        carriageLoop.setFeederSpeed(1);
      } else if (Robot.m_oi.getToolLeftTrigger()) {
        shooterSpeed = -0.40;
        carriageLoop.setFeederSpeed(0);
        carriageLoop.setIsShooting(true);
      }
      // } else { //no shot
      // shooterSpeed = 0;
      // //carriageLoop.setIsShooting(false);
      // // carriageLoop.setFeederSpeed(0);
      // }

      // set pneumatics
      carriageLoop.setClawPos(!clawToggle.get());
      carriageLoop.setSliderPos(!sliderToggle.get());
      carriageLoop.setEjectorPos(ejectToggle.get());

      // set shooter
      carriageLoop.setShooterSpeed(shooterSpeed);
      carriageLoop.setCarriageState(CarriageControlState.OPEN_LOOP);
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