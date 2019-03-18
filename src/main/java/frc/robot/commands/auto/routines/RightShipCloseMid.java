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
    addParallel(new SetXY(FieldPoints.RIGHT_LEVEL_2));
    addSequential(new YeetOffSequence());

    //addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));

    //
    addSequential(new SetXY(FieldPoints.RIGHT_OFF_PLATFORM));
    addSequential(new DriveToGoal(FieldPoints.CLOSE_RIGHT_CARGO_PRE_SCORE, 8, 1, false));
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION + 4, NumberConstants.ELEVATOR_MAX_SPEED, 0.3, 1));

    addSequential(new DriveTurn(-90, 1, 5));
    addSequential(new DriveTurn(1, 4, true, false));

    addSequential(new SetTrayCommand(false)); 
    addSequential(new DriveDistance(38, 0, 0.35, 10 + FieldPoints.CARGO_SHIP_EJECT_DIST, true, false)); 
    addSequential(new EjectHatchSequence());
  
    addSequential(new SetXY(FieldPoints.CLOSE_RIGHT_CARGO));
    addSequential(new DriveToGoal(FieldPoints.PRE_FEEDER_RIGHT, 5, 1, false));
    addSequential(new SetClawCommand(true));
    addParallel(new DriveDistance(80, 0, 0.5, 6, true, false));
    addSequential(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1)); 
    addSequential(new SetClawCommand(false));
    addSequential(new SetXY(FieldPoints.RIGHT_FEEDER));
    addSequential(new DriveToGoal(FieldPoints.MID_RIGHT_CARGO_PRE_SCORE, 9, 0.6, true, true));

    addSequential(new DriveTurn(90, 1, 5));
    addSequential(new DriveTurn(1, 4, true, false));

    addSequential(new SetTrayCommand(false)); 
    addSequential(new DriveDistance(38, 0, 0.35, 10 + FieldPoints.CARGO_SHIP_EJECT_DIST, true, false)); 
    addSequential(new EjectHatchSequence());
    
  }

}
