/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.NumberConstants;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.commands.auto.EjectHatchSequence;
import frc.robot.commands.auto.WaitCommand;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.carriage.SetTrayCommand;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.util.FieldPoints;
import frc.robot.commands.auto.YeetOffSequence;


public class LeftShipCloseRocket extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftShipCloseRocket() {
   
    //score front hatch
    addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));
    addSequential(new DriveDistance(160, 17, 1, 10));
    addSequential(new DriveTurn(-3, 1, 4));
    addSequential(new DriveTurn(1, 3.5, true));
    addSequential(new DriveDistance(100, 0, 0.35, FieldPoints.CARGO_SHIP_EJECT_DIST + 5, true));
    addSequential(new EjectHatchSequence());
    addSequential(new DriveDistance(-30, 0, 0.5, 16));

    //get second hatch
    addSequential(new DriveDistance(-160, 50, 1, 30));
    addSequential(new DriveTurn(175, 1, 5));
    addSequential(new DriveDistance(150, 0, 0.45, FieldPoints.FEEDER_EJECT_DIST, true)); 
    addSequential(new SetClawCommand(false));
    addSequential(new WaitCommand(0.05));

    addSequential(new DriveDistance(-100, 190, 1, 16, 0, false));
    addParallel(new SetTrayCommand(false));
    addSequential(new DriveTurn(-20, 1, 5));
    addSequential(new DriveDistance(80, 0, 0.45, FieldPoints.ROCKET_EJECT_DIST, true)); 
    addSequential(new EjectHatchSequence());



  }
}
