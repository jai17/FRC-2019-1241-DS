/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.commands.auto.EjectHatchSequence;
import frc.robot.commands.auto.WaitCommand;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.carriage.SetTrayCommand;
import frc.robot.util.FieldPoints;

public class LeftShipFrontClose extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftShipFrontClose() {
    //score front hatch
    // addSequential(new DriveDistance(168, 12, 1, 10)); //old drive to first
    addSequential(new DriveDistance(160, 13, 1, 10));
    addSequential(new DriveTurn(-3, 1, 4));
    addSequential(new DriveDistance(100, 0, 0.35, FieldPoints.CARGO_SHIP_EJECT_DIST + 5, true));
    addSequential(new EjectHatchSequence());
    addSequential(new DriveDistance(-30, 0, 0.5, 16));

    //get second hatch
    addSequential(new DriveDistance(-180, 50, 1, 30));
    addSequential(new DriveTurn(175, 1, 5));
    addSequential(new DriveDistance(150, 0, 0.45, FieldPoints.FEEDER_EJECT_DIST, true)); 
    addSequential(new SetClawCommand(false));
    addSequential(new WaitCommand(0.05));

    //Drive Back to second cargo bay
    // addSequential(new DriveDistance(-270, 196, 1, 30, 0, true));
    // addSequential(new DriveDistance(-44, 196, 0.8, 10, 0, false));
    addSequential(new DriveDistance(-304, 194, 1, 30, 0, false));

    addParallel(new SetTrayCommand(false));
    addSequential(new DriveTurn(96, 0.5, 3));
    addSequential(new DriveTurn(0.6, 3.5, true));
    //Drive Track to cargo bay
    addSequential(new DriveDistance(60, 0, 0.28, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
    //Eject Hatch
    addSequential(new EjectHatchSequence());
    addSequential(new WaitCommand(0.03));
    //back up
    addSequential(new DriveDistance(-30, 90, 0.5, 15, false));
    //drive to depot and celebrate your two hatches
    addSequential(new DriveTurn(190, 0.75, 30));
    addSequential(new DriveDistance(190, 190, 1, 30, 0, false));
  }
}
