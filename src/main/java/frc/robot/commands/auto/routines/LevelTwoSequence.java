/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.NumberConstants;
import frc.robot.commands.cargo.SetIntakePivotCommand;
import frc.robot.commands.drive.SetDingusCommand;
import frc.robot.commands.drive.TimedDrive;

public class LevelTwoSequence extends CommandGroup {
  
  private static LevelTwoSequence mInstance;

  public static LevelTwoSequence getInstance() {
    if (mInstance == null) {
      mInstance = new LevelTwoSequence();
    }
    return mInstance;
  }

  public LevelTwoSequence() {
    //bintake down
    addSequential(new SetIntakePivotCommand(NumberConstants.CARGO_LIFTING_ANGLE, NumberConstants.CARGO_MAX_SPEED, 0.5, 3));
    // //drive at 40 percent
    addSequential(new TimedDrive(1.5, 0.75));
    addSequential(new SetDingusCommand(true));
    addSequential(new SetIntakePivotCommand(NumberConstants.CARGO_RESTING_ANGLE, NumberConstants.CARGO_MAX_SPEED, 0.5, 1));

    addSequential(new TimedDrive(3, 0.65));
     // //stop drive after time
    addSequential(new SetDingusCommand(false));
  }
}
