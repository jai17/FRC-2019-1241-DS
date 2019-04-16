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
import frc.robot.commands.auto.TurnToGoal;
import frc.robot.commands.auto.SetXY;
import frc.robot.commands.auto.YeetOffSequence;
import frc.robot.commands.carriage.SetClawCommand;
import frc.robot.commands.carriage.SetTrayCommand;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.util.FieldPoints;

public class RightShipCloseMid extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RightShipCloseMid() {

    // //Get off Level 2
addSequential(new YeetOffSequence());
//Drive to close cargo bay
addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION+3, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
addSequential(new DriveDistance (157, 5, 1, 15, 0, false));
//Extend Tray while turning to close cargo bay
addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(-90, 0.75, 5));
addSequential(new DriveTurn (0.5, 3.5, true));
//Drive Track to cargo bay
addSequential(new DriveDistance(60, 0, 0.25, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
//addSequential(new DriveDistance (46 , -90, 1, 4));
//Eject Hatch
addSequential(new EjectHatchSequence());

//Drive Back 
addSequential(new DriveDistance (-40 , -90, 1, 12, 0, false));
//Turn to Feeder
addSequential(new DriveTurn(-190, 1, 15));
//Drive To Feeder
addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
addSequential(new DriveDistance (215, -190, 0.90, 20, 0, true));
//Turn to Feeder
addSequential(new DriveTurn(-180, 1, 8));
//Drive Track To Feeder
addSequential(new DriveDistance(150, 0, 0.6, FieldPoints.FEEDER_EJECT_DIST, true)); 
//addSequential(new DriveDistance (42 , -180, 1, 4));
//Close Fingers 
addSequential(new SetClawCommand(false));

//Drive Back to second cargo bay
addSequential(new DriveDistance (-287, -195, 0.8, 10, 0, true));
//Extend Tray while turning to close cargo bay
addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(-75, 0.3, 6));
addSequential(new DriveTurn (0.3, 3, true));
//Drive Track to cargo bay
addSequential(new DriveDistance(60, 0, 0.3, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
//addSequential(new DriveDistance (46 , -90, 1, 4));
//Eject Hatch
addSequential(new EjectHatchSequence());


/*Old
//HIGH PID FALSE IN DRIVETURN
//addParallel(new SetXY(FieldPoints.RIGHT_LEVEL_2));
//Get off Level 2
//addSequential(new YeetOffSequence());
//Drive to close cargo bay
addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
addSequential(new DriveDistance (167 , 11, 1, 6));
//Extend Tray while turning to close cargo bay
addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(-90, 1, 5));
//Drive Track to cargo bay
addSequential(new DriveDistance(50, 0, 0.3, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
//addSequential(new DriveDistance (46 , -90, 1, 4));
//Eject Hatch
addSequential(new EjectHatchSequence());
//Drive Back 
addSequential(new DriveDistance (-46 , -90, 1, 6));
//Turn to Feeder
addSequential(new DriveTurn(-197, 1, 4));
//Drive To Feeder
addSequential(new DriveDistance (209 , -194, 1, 5));
//Turn to Feeder
addSequential(new DriveTurn(-180, 1, 4));
//Drive Track To Feeder
addSequential(new DriveDistance(100, 0, 0.5, FieldPoints.FEEDER_EJECT_DIST, true)); 
//addSequential(new DriveDistance (42 , -180, 1, 4));
//Close Fingers 
addSequential(new SetClawCommand(false));
//Drive Back to second cargo bay
addSequential(new DriveDistance (-280 , -192, 1, 6));
//Extend Tray while turning to close cargo bay
//addParallel(new SetTrayCommand(false));
addSequential(new DriveTurn(-90, 1, 6));
*/
//OLD


//Drive Track to cargo bay
//addSequential(new DriveDistance(50, 0, 0.3, FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
//addSequential(new DriveDistance (46 , -90, 1, 4));
//Eject Hatch
//addSequential(new EjectHatchSequence());

    /*
    addParallel(new SetXY(FieldPoints.RIGHT_LEVEL_2));
    addSequential(new YeetOffSequence());

    //addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));

    //
    addSequential(new SetXY(FieldPoints.RIGHT_OFF_PLATFORM));
    addSequential(new DriveToGoal(FieldPoints.CLOSE_RIGHT_CARGO_PRE_SCORE, 8, 1, false));
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION + 4, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));

    addSequential(new DriveTurn(-90, 1, 5));
    addSequential(new DriveTurn(1, 4, true));

    addSequential(new SetTrayCommand(false)); 
    addSequential(new DriveDistance(38, 0, 0.35, 10 + FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
    addSequential(new EjectHatchSequence());
  
    addSequential(new SetXY(FieldPoints.CLOSE_RIGHT_CARGO));
    addSequential(new DriveToGoal(FieldPoints.PRE_FEEDER_RIGHT, 5, 1, false));
    addSequential(new SetClawCommand(true));
    addParallel(new DriveDistance(80, 0, 0.5, 6, true));
    addSequential(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1)); 
    addSequential(new SetClawCommand(false));
    addSequential(new SetXY(FieldPoints.RIGHT_FEEDER));
    addSequential(new DriveToGoal(FieldPoints.MID_RIGHT_CARGO_PRE_SCORE, 9, 0.6, true, true));

    addSequential(new DriveTurn(90, 1, 5));
    addSequential(new DriveTurn(1, 4, true));

    addSequential(new SetTrayCommand(false)); 
    addSequential(new DriveDistance(38, 0, 0.35, 10 + FieldPoints.CARGO_SHIP_EJECT_DIST, true)); 
    addSequential(new EjectHatchSequence());
    */
    // addParallel(new SetXY(FieldPoints.LEFT_LEVEL_2));
    // addSequential(new YeetOffSequence());

    // //addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));

    // //
    // addSequential(new SetXY(FieldPoints.LEFT_OFF_PLATFORM));
    // addSequential(new DriveToGoal(FieldPoints.CLOSE_LEFT_CARGO_PRE_SCORE, 8, 1, false));
    // addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION + 4, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));

    // addSequential(new DriveTurn(-90, 1, 5));
    // addSequential(new DriveTurn(1, 4, true, false));

    // addSequential(new SetTrayCommand(false)); 
    // addSequential(new DriveDistance(38, 0, 0.35, 10 + FieldPoints.CARGO_SHIP_EJECT_DIST, true, false)); 
    // addSequential(new EjectHatchSequence());
  
    // addSequential(new SetXY(FieldPoints.CLOSE_LEFT_CARGO));
    // addSequential(new DriveToGoal(FieldPoints.PRE_FEEDER_LEFT, 5, 1, false));
    // addSequential(new SetClawCommand(true));
    // addParallel(new DriveDistance(80, 0, 0.5, 6, true, false));
    // addSequential(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1)); 
    // addSequential(new SetClawCommand(false));
    // addSequential(new SetXY(FieldPoints.LEFT_FEEDER));
    // addSequential(new DriveToGoal(FieldPoints.MID_LEFT_CARGO_PRE_SCORE, 9, 0.6, true, true));

    // addSequential(new DriveTurn(90, 1, 5));
    // addSequential(new DriveTurn(1, 4, true, false));

    // addSequential(new SetTrayCommand(false)); 
    // addSequential(new DriveDistance(38, 0, 0.35, 10 + FieldPoints.CARGO_SHIP_EJECT_DIST, true, false)); 
    // addSequential(new EjectHatchSequence());
  }

}
