/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.carriage;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.loops.CarriageLoop;
import frc.robot.subsystems.Carriage;

public class SetEjectorCommand extends Command {

  Carriage carriage;
  CarriageLoop carriageLoop;
  boolean forward;

  public SetEjectorCommand(boolean forward) {
    carriage = Carriage.getInstance();
    carriageLoop = CarriageLoop.getInstance();

    this.forward = forward;
    requires(carriage);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (forward){
      carriage.ejectHatch();
    } else {
      carriage.retractEjector();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
