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
import frc.robot.commands.auto.DriveToGoal;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.commands.auto.EjectHatchSequence;
import frc.robot.commands.auto.ElevatorSetpointWait;
import frc.robot.commands.auto.SetXY;
import frc.robot.commands.auto.TurnToGoal;
import frc.robot.commands.auto.YeetOffSequence;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.carriage.SetTrayCommand;
import frc.robot.commands.drive.ResetDriveEncoder;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.util.FieldPoints;
import frc.robot.commands.auto.WaitCommand;
import frc.robot.Robot;

public class LeftShipCloseMid extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftShipCloseMid() {

// //Get off Level 2
// addSequential(new YeetOffSequence());
//Drive to close cargo bay
addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION+3, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
addSequential(new DriveDistance (256, -5, 1, 20, 0, false));
//Extend Tray while turning to close cargo bay
// addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(90, 1, 5));
addSequential(new DriveTurn (0.6, 3.5, true));
//Drive Track to cargo bay
addSequential(new DriveDistance(60, 0, 0.35, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
//Eject Hatch
addSequential(new EjectHatchSequence());

//Drive Back 
addSequential(new DriveDistance (-50, 90, 1, 20, 0, false));
//Turn to Feeder
addSequential(new DriveTurn(190, 1, 15));
//Drive To Feeder
addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
// addSequential(new DriveDistance (240, 191, 0.90, 20, 0, true));
// addSequential(new DriveDistance (30, 191, 0.90, 20, 0, false));
addSequential(new DriveDistance(245, 190, 1, 30, 0, false));
//Turn to Feeder
addSequential(new DriveTurn(180, 0.9, 5));
// addSequential(new DriveTurn(0.5, 6, true));
//Drive Track To Feeder
addSequential(new DriveDistance(150, 0, 0.50, FieldPoints.FEEDER_EJECT_DIST, true)); 
//addSequential(new DriveDistance (42 , -180, 1, 4));
//Close Fingers 
addSequential(new SetClawCommand(false));
addSequential(new WaitCommand(0.05));

//Drive Back to second cargo bay
// addSequential(new DriveDistance(-300, 195, 0.8, 30, 0, true));
// addSequential(new DriveDistance(-30, 195, 0.8, 10, 0, false));
addSequential(new DriveDistance(-310, 195, 1, 30, 0, false));
//Extend Tray while turning to close cargo bay
// addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(82, 0.75, 3));
addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(0.6, 2, true));
// addSequential(new DriveTurn(90, 0.75, 5));
// addSequential(new DriveTurn (0.5, 2.5, true));

//Drive Track to cargo bay
addSequential(new DriveDistance(60, 0, 0.35, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
//addSequential(new DriveDistance (46 , -90, 1, 4));
//Eject Hatch
addSequential(new EjectHatchSequence());
addSequential(new DriveDistance(-30, 90, 0.5, 5, false));

   
    //addParallel(new SetXY(FieldPoints.LEFT_LEVEL_2));
    // addSequential(new YeetOffSequence());

    // //For Without Yeet
    // //addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));

    // //
    // addSequential(new SetXY(FieldPoints.LEFT_OFF_PLATFORM));
    // addSequential(new DriveToGoal(FieldPoints.CLOSE_LEFT_CARGO_PRE_SCORE, 8, 1, false));
    // addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION + 4, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));

    // addSequential(new TurnToGoal(FieldPoints.CLOSE_LEFT_CARGO, 3, 0.9));
    // addSequential(new DriveTurn(1, 4, true));

    // addSequential(new SetTrayCommand(false)); 
    // addSequential(new DriveDistance(38, 0, 0.35, 10 + FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
    // addSequential(new EjectHatchSequence());
  
    // addSequential(new SetXY(FieldPoints.CLOSE_LEFT_CARGO));
    // addSequential(new DriveDistance (-20, 90, 1, 3));
    // addSequential(new TurnToGoal(FieldPoints.LEFT_FEEDER, 4, 1));
    // addSequential(new DriveToGoal(FieldPoints.PRE_FEEDER_LEFT, 5, 1, false));
    // addSequential(new SetClawCommand(true));
    // addParallel(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1)); 
    // addSequential(new DriveDistance(80, 0, 0.5, 6, true));
    // addSequential(new SetClawCommand(false));
    // addSequential(new SetXY(FieldPoints.LEFT_FEEDER));
    // addSequential(new WaitCommand(0.25));
    // addSequential(new DriveToGoal(FieldPoints.MID_LEFT_CARGO_PRE_SCORE, 9, 0.6, true, true));

    // addSequential(new TurnToGoal(FieldPoints.MID_LEFT_CARGO, 3, 0.9));
    // addSequential(new DriveTurn(1, 4, true));

    // addSequential(new SetTrayCommand(false)); 
    // addSequential(new DriveDistance(38, 0, 0.35, 10 + FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
    // addSequential(new EjectHatchSequence());

  }
}