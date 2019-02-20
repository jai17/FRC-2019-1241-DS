/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.NumberConstants;
import frc.robot.commands.auto.WaitCommand;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.elevator.ElevatorSetpoint;

public class HatchFinesse extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HatchFinesse() {
    //addSequential(new SetClawCommand(false));
    addSequential(new ElevatorSetpoint(NumberConstants.HATCH_FINESSE_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.4, 1));
    addSequential(new WaitCommand(0.5));
    addSequential(new SetClawCommand(true));
    addSequential(new WaitCommand(0.25));
    addSequential(new SetClawCommand(false));
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 0.2));
    System.out.println(this.toString() + " HAS FINISHED");
  }
}
