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
import frc.robot.commands.auto.EjectHatchSequence;
import frc.robot.commands.carriage.SetTrayCommand;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.util.FieldPoints;

public class DriveTrackTest extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DriveTrackTest() {
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
    addSequential(new SetTrayCommand(false));
    addSequential(new DriveDistance(100, 0, 0.4, FieldPoints.ROCKET_EJECT_DIST, true));
    addSequential(new EjectHatchSequence());
  }
}
