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
import frc.robot.commands.auto.EjectHatchRocketSequence;
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
    addSequential(new DriveToGoal(FieldPoints.RIGHT_CLOSE_ROCKET, 6, 1, false));
    
    addParallel(new SetTrayCommand(false));
    addSequential(new DriveTurn(0.8, 5, true));

    addParallel(new DriveDistance(40, 0, 0.25, 3, true)); 
    addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_MID_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.25, 1));
    
    addSequential(new EjectHatchRocketSequence());
    addSequential(new SetXY(FieldPoints.CLOSE_RIGHT_ROCKET_SCORE));

    addParallel(new DriveToGoal(FieldPoints.PRE_FEEDER_RIGHT, 5, 0.75, true));
    addSequential(new ElevatorSetpointWait(0.5, NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.35, 1));
    
    addSequential(new TurnToGoal(FieldPoints.PRE_FEEDER_RIGHT, 8, 0.75));
    addSequential(new DriveDistance(60, 0, 0.25, 10, true)); 
    addSequential(new SetClawCommand(false));
  }
}
