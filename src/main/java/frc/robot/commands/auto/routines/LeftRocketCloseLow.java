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

public class LeftRocketCloseLow extends CommandGroup {
  
  public LeftRocketCloseLow() {
    addParallel(new SetXY(FieldPoints.LEFT_LEVEL_2));
    addSequential(new YeetOffSequence());

      // For without Yeet
    //addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));
    
    addSequential(new SetXY(FieldPoints.LEFT_OFF_PLATFORM));
    addSequential(new DriveToGoal(FieldPoints.LEFT_CLOSE_ROCKET, 6, 1, false));
    
    addParallel(new SetTrayCommand(false));
    addSequential(new DriveTurn(0.6, 5, true, false));

    addParallel(new DriveDistance(40, 0, 0.4, FieldPoints.ROCKET_EJECT_DIST, true)); 
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_MID_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
    
    addSequential(new EjectHatchSequence());
    addSequential(new SetXY(FieldPoints.CLOSE_LEFT_ROCKET_SCORE));

    addParallel(new DriveToGoal(FieldPoints.PRE_FEEDER_LEFT, 4, 1, true));
    addSequential(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1));
    

    addSequential(new TurnToGoal(FieldPoints.LEFT_FEEDER, 4, 0.9));
    addSequential(new SetClawCommand(true));
    addSequential(new DriveDistance(80, 0, 0.25, 8, true)); 
    addSequential(new SetClawCommand(false));
    addParallel(new DriveToGoal(FieldPoints.LEFT_FAR_ROCKET, 4, 1, true));


    //addParallel(new DriveToGoal(FieldPoints.LEFT_FAR_ROCKET, 4, 1, true));

    // For without Yeet
    // addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));
    
    // addSequential(new SetXY(FieldPoints.LEFT_OFF_PLATFORM));
    // addSequential(new DriveToGoal(FieldPoints.LEFT_CLOSE_ROCKET, 6, 0.8, false));
    
    // addParallel(new SetTrayCommand(false));
    // addSequential(new DriveTurn(0, 1, 2, 5));

    // addSequential(new DriveTurn(0.5, 5, true, false));

    // addParallel(new DriveDistance(40, 0, 0.4, FieldPoints.ROCKET_EJECT_DIST, true, false)); 
    // addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_MID_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
    
    // addSequential(new EjectHatchSequence());
    // addSequential(new SetXY(FieldPoints.CLOSE_LEFT_ROCKET_SCORE));

    // addSequential(new DriveToGoal(FieldPoints.PRE_FEEDER_LEFT, 4, 1, true));
    // addSequential(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1));
    
    // addSequential(new TurnToGoal(FieldPoints.LEFT_FEEDER, 4, 0.9));
    // addSequential(new DriveDistance(60, 0, 0.25, 10, true, false)); 
    // addSequential(new SetClawCommand(false));
  }
}
