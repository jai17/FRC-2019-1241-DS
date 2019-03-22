/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.NumberConstants;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.loops.CarriageLoop;

public class HatchFeedSequence extends CommandGroup {
  
  CarriageLoop carriageLoop = CarriageLoop.getInstance();

  public static HatchFeedSequence mInstance;

  public static HatchFeedSequence getInstance() {
    if (mInstance == null) {
      mInstance = new HatchFeedSequence();
    }
    return mInstance;
  }

  public HatchFeedSequence() {
    // // bring elevator to height, point fingers
    addSequential(new ElevatorSetpoint(NumberConstants.HINTAKE_FEEDING_HEIGHT, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));
    // System.out.println("Elevator to hintake height");
    addSequential(new SetClawCommand(true));
    // System.out.println("claw pointed");

    // //bring hatch pivot to feeding height, open fingers
    addSequential(new SetHatchPivotCommand(NumberConstants.HATCH_FEEDING_ANGLE, NumberConstants.HATCH_MAX_SPEED - 400, 1, 3));
    // System.out.println("hatch up");
    addSequential(new SetClawCommand(false));
    // System.out.println("claw opened");

    // //bring elevator up for hintake to clear
    addSequential(new ElevatorSetpoint(NumberConstants.HINTAKE_FEEDING_HEIGHT + 6, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));
    // System.out.println("elevator clearing");

    // //bring to rest angle, bring elevator back down
    addSequential(new SetHatchPivotCommand(NumberConstants.HATCH_REST_ANGLE, NumberConstants.HATCH_MAX_SPEED, 0.2, 0.5));
    // System.out.println("hatch pivot back in");
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));
    // System.out.println("elevator low hatch");
  }
}