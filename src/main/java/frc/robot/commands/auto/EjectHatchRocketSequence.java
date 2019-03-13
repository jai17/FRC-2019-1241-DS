/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.carriage.SetEjectorCommand;
import frc.robot.commands.carriage.SetTrayCommand;

public class EjectHatchRocketSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public EjectHatchRocketSequence() {
    addSequential(new SetClawCommand(true));
    addSequential(new WaitCommand(0.05));
    addSequential(new SetEjectorCommand(true));
    addSequential(new WaitCommand(0.05));
    addParallel(new SetEjectorCommand(false));
    addSequential(new SetTrayCommand(true));
    addSequential(new WaitCommand(0.5));
  }
}
