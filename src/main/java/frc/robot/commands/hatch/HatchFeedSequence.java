/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.NumberConstants;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.elevator.ElevatorSetpoint;

public class HatchFeedSequence extends CommandGroup {
  
  public HatchFeedSequence() {
    //bring elevator to height, point fingers
    addSequential(new ElevatorSetpoint(NumberConstants.HINTAKE_FEEDING_HEIGHT, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));
    addParallel(new SetClawCommand(true));

    //bring hatch pivot to feeding height, open fingers
    addSequential(new SetHatchPivotCommand(NumberConstants.HATCH_FEEDING_ANGLE, NumberConstants.HATCH_MAX_SPEED, 0.1, 0.5));
    addSequential(new SetClawCommand(false));

    //bring elevator up for hintake to clear
    addParallel(new ElevatorSetpoint(NumberConstants.HINTAKE_FEEDING_HEIGHT + 6, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));

    //bring to rest angle, bring elevator back down
    addSequential(new SetHatchPivotCommand(NumberConstants.HATCH_REST_ANGLE, NumberConstants.HATCH_MAX_SPEED, 0.1, 0.5));
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_REST_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));
  }
}
