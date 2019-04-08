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
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.util.FieldPoints;
import frc.robot.commands.auto.WaitCommand;
import frc.robot.Robot;

public class LeftShipCloseMid extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftShipCloseMid() {

//Get off Level 2
//addSequential(new YeetOffSequence());
//Drive to close cargo bay
addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
addSequential(new DriveDistance (167 , -11, 1, 6));
//Extend Tray while turning to close cargo bay
addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(90, 1, 5));
//Drive Track to cargo bay
addSequential(new DriveDistance(50, 0, 0.3, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
//addSequential(new DriveDistance (46 , -90, 1, 4));
//Eject Hatch
addSequential(new EjectHatchSequence());
//Drive Back 
addSequential(new DriveDistance (-46 , 90, 1, 6));
//Turn to Feeder
addSequential(new DriveTurn(197, 1, 4));
//Drive To Feeder
addSequential(new DriveDistance (209 , 194, 1, 5));
//Turn to Feeder
addSequential(new DriveTurn(180, 1, 4));
//Drive Track To Feeder
addSequential(new DriveDistance(100, 0, 0.5, FieldPoints.FEEDER_EJECT_DIST, true)); 
//addSequential(new DriveDistance (42 , -180, 1, 4));
//Close Fingers 
addSequential(new SetClawCommand(false));
//Drive Back to second cargo bay
addSequential(new DriveDistance (-280 , 192, 1, 6));
//Extend Tray while turning to close cargo bay
//addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(90, 1, 6));
//Drive Track to cargo bay
//addSequential(new DriveDistance(50, 0, 0.3, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
//addSequential(new DriveDistance (46 , -90, 1, 4));
//Eject Hatch
//addSequential(new EjectHatchSequence());

   
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