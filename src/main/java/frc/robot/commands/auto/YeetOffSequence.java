/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;
import frc.robot.NumberConstants;
import frc.robot.Robot; 

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.elevator.ElevatorSetpoint;

public class YeetOffSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public YeetOffSequence() {
    addSequential(new DriveDistance(84, 0, 1, 15));
    addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));
  }
}
