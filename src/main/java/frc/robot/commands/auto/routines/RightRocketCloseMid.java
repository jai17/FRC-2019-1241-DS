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

public class RightRocketCloseMid extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RightRocketCloseMid() {
    
    // addParallel(new SetXY(FieldPoints.RIGHT_LEVEL_2));
    // addSequential(new YeetOffSequence());
    //For without Yeet
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));
    
    addSequential(new SetXY(FieldPoints.RIGHT_OFF_PLATFORM));
    addSequential(new DriveToGoal(FieldPoints.RIGHT_CLOSE_ROCKET, 7, 1, false));
    
    addParallel(new SetTrayCommand(false));
    //addSequential(new TurnToGoal(FieldPoints.RIGHT_ROCKET, 4, 0.9));
    addSequential(new DriveTurn(0.6, 5, true));
    addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
    addSequential(new DriveDistance(50, 0, 0.3, FieldPoints.ROCKET_EJECT_DIST, true)); 
    
    addSequential(new EjectHatchSequence());
    addSequential(new SetXY(FieldPoints.CLOSE_RIGHT_ROCKET_SCORE));

    addParallel(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1));
    addSequential(new DriveToGoal(FieldPoints.PRE_FEEDER_RIGHT, 4, 1, true));

    addSequential(new TurnToGoal(FieldPoints.RIGHT_FEEDER, 4, 0.9));
    
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));

    addSequential(new SetClawCommand(true));
    addSequential(new DriveDistance(80, 0, 0.4, FieldPoints.FEEDER_EJECT_DIST, true)); 
    addSequential(new SetClawCommand(false));
    addSequential(new SetXY(FieldPoints.RIGHT_FEEDER_ROBOT));

    addParallel(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1));
    addSequential(new DriveDistance (-285, -186, 1, 3));
    // addSequential(new DriveToGoal(FieldPoints.RIGHT_FAR_ROCKET, 4, 1, false));

    // addSequential(new TurnToGoal(FieldPoints.RIGHT_ROCKET, 4, 0.9));

    // addSequential(new DriveTurn(0.6, 4, true));
    // addSequential(new SetTrayCommand(false));
    // addSequential(new DriveDistance(80, 0, 0.15, FieldPoints.ROCKET_EJECT_DIST, true));
    // addSequential(new EjectHatchSequence());
    // addSequential(new SetXY(FieldPoints.RIGHT_FAR_ROCKET_SCORE));
  }
}
