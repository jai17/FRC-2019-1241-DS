/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.NumberConstants;
import frc.robot.Robot;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveToGoal;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.commands.auto.EjectHatchSequence;
import frc.robot.commands.auto.ElevatorSetpointWait;
import frc.robot.commands.auto.SetXY;
import frc.robot.commands.auto.TrackDrive;
import frc.robot.commands.auto.TurnToGoal;
import frc.robot.commands.auto.WaitCommand;
import frc.robot.commands.auto.YeetOffSequence;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.carriage.SetTrayCommand;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.util.FieldPoints;

public class RightRocketCloseMid extends CommandGroup {
  /**
   * TO-DO:
   */
  public RightRocketCloseMid() {
    // addSequential(new DriveTurn(60, 0.5, 2));
    // addSequential(new TrackDrive(false));
    // addSequential(new DriveDistance(-30, Robot.drive.getAngle(), 0.5, 6));

    addSequential(new YeetOffSequence());
    // //Drive to close rocket 
    // addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION,
    //               NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));
    addSequential(new DriveDistance(115, 57, 1, 10, 0, false));
    // //Turning to close rocket while extending tray
    addParallel(new SetTrayCommand(false));
    addSequential(new DriveTurn(25, 1, 4));
    addSequential(new DriveTurn(0.7, 3.5, true));

    // //Drive Track to close rocket
    addSequential(new DriveDistance(80, 0, 0.35, FieldPoints.ROCKET_EJECT_DIST, true));
    // //Eject Hatch
    addSequential(new EjectHatchSequence());
    //addSequential(new WaitCommand(0.1));
    // //Drive Back
    addSequential(new DriveDistance(-25, 33, 1, 20, 0, false));
    // //Turn to Feeder
    addSequential(new DriveTurn(173, 1, 4));
    // //Drive To Feeder
    addSequential(new DriveDistance(140, 173, 1, 15, 0, false));
    // //Turn to Feeder

    // addSequential(new DriveTurn(180, 1, 4));
    // addSequential(new DriveTurn(1, 6, true));
    // // //Drive Track To Feeder
    // addSequential(new DriveDistance(200, 0, 0.7, FieldPoints.FEEDER_EJECT_DIST, true));
    // // Close Fingers
    // addSequential(new SetClawCommand(false));

    addSequential(new DriveTurn(180, 1, 6));
    addSequential(new DriveTurn(1, 3.5, true));
    // addSequential(new DriveTurn(0.5, 6, true));
    //Drive Track To Feeder
    addSequential(new DriveDistance(200, 0, 0.7, FieldPoints.FEEDER_EJECT_DIST + 3, true)); 
    
    addSequential(new SetClawCommand(false));
    addSequential(new WaitCommand(0.15));
    // Drive Back to far rocket
    addSequential(new DriveDistance(-260, 167, 0.75, 25, 0, true));
    addSequential(new DriveDistance(-90, 205, 1, 10, 0, false));
    // Extend Tray while turning to close cargo bay
    addParallel(new SetTrayCommand(false));
    addSequential(new DriveTurn(148, 1, 6));
    addSequential(new DriveTurn(1, 3, true));
    // Drive Track to cargo bay
    addSequential(new DriveDistance(60, 0, 0.3, FieldPoints.ROCKET_EJECT_DIST, true));
    // Eject Hatch
    addSequential(new EjectHatchSequence());
    addSequential(new DriveDistance(-50, 160, 1, 30, 0, true));
    addSequential(new DriveDistance(-60, 90, 1, 30, 0, true));
    addSequential(new DriveDistance(-200, 0, 1, 20, 1, true));

    //addSequential(new DriveDistance(-40, Robot.drive.getAngle(), 1, 10, 1, false));
  }
}
