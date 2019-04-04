/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.NumberConstants;
import frc.robot.Robot;
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

  //Elevator (parallel) and drive nested (sequential)
  
  public LeftRocketCloseLow() {
    // addParallel(new SetXY(FieldPoints.LEFT_LEVEL_2));
    // addSequential(new YeetOffSequence());

    // For without Yeet
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));
    
    addSequential(new SetXY(FieldPoints.LEFT_OFF_PLATFORM));
    addSequential(new DriveToGoal(FieldPoints.LEFT_CLOSE_ROCKET, 7.5, 0.9, false));
    
    addParallel(new SetTrayCommand(false));
    addSequential(new DriveTurn(0.6, 5, true));

    addParallel(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
    addSequential(new DriveDistance(50, 0, 0.2, FieldPoints.ROCKET_EJECT_DIST, true));
    
    addSequential(new EjectHatchSequence());
    addSequential(new SetXY(FieldPoints.CLOSE_LEFT_ROCKET_SCORE));
    addSequential(new DriveToGoal(FieldPoints.POST_LEFT_CLOSE_ROCKET, 4, 1, true));
    // addSequential(new SetXY(FieldPoints.POST_LEFT_CLOSE_ROCKET));

    addParallel(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1));
    addSequential(new DriveToGoal(FieldPoints.PRE_FEEDER_LEFT, 4, 1, true));
    
    addSequential(new TurnToGoal(FieldPoints.LEFT_FEEDER, 3, 0.8));
    addSequential(new DriveDistance(100, 0, 0.25, FieldPoints.FEEDER_EJECT_DIST, true)); 
    addSequential(new SetClawCommand(false));
    addSequential(new SetXY(FieldPoints.LEFT_FEEDER_ROBOT));

    // // addSequential(new DriveToGoal(FieldPoints.LEFT_MID_ROCKET, 4, 1, true));
    addSequential(new DriveToGoal(FieldPoints.LEFT_FAR_ROCKET, 4, 1, true));
    addSequential(new TurnToGoal(FieldPoints.LEFT_ROCKET, 4, 0.9));

    addSequential(new DriveTurn(0.6, 4, true));
    addSequential(new SetTrayCommand(false));
    addSequential(new DriveDistance(80, 0, 0.15, FieldPoints.ROCKET_EJECT_DIST, true));
    addSequential(new SetXY(FieldPoints.LEFT_FAR_ROCKET_SCORE));
    addSequential(new EjectHatchSequence());
    addSequential(new DriveToGoal(FieldPoints.POST_LEFT_FAR_ROCKET, 4, 1, true));
    addSequential(new SetXY(FieldPoints.POST_LEFT_FAR_ROCKET));

    addSequential(new DriveToGoal(FieldPoints.PRE_CLOSE_LEFT_CARGO, 4, 1, true));

  
  }

  private static class nested extends CommandGroup {
    public nested(){
      
    }
  }
}
