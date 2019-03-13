/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;

public class ElevatorSetpointWait extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorSetpointWait(double delay, double setpoint, int cruiseVelocity, double timeToMax, double timeOut) {
    addSequential(new WaitCommand(delay));
    addSequential(new ElevatorSetpoint(setpoint, cruiseVelocity, timeToMax, timeOut));
  }
}
