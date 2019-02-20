/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.NumberConstants;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.carriage.SetEjectorCommand;
import frc.robot.commands.carriage.SetTrayCommand;

public class FarRightShipHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FarRightShipHatch() {
    addSequential(new HatchFinesse());
    addSequential(new DriveDistance(20, 0, 1, 2));
    addSequential(new DriveTurn(30, 1, 2));
    addSequential(new DriveDistance(230, 30, 1, 2));
    addSequential(new DriveTurn(-90, 1, 1));
    addSequential(new DriveDistance(50, -90, 0, 0.5, 2, true));
    addSequential(new SetClawCommand(true));
    addSequential(new SetEjectorCommand(true));
  }
}
