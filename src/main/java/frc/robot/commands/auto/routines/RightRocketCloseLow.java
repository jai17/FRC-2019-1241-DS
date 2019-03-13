/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.YeetOffSequence;
import frc.robot.commands.carriage.SetTrayCommand;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.auto.SetXY;
import frc.robot.NumberConstants;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveToGoal;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.commands.auto.TurnToGoal;
import frc.robot.util.FieldPoints;

public class RightRocketCloseLow extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RightRocketCloseLow() {
    addParallel(new SetXY(FieldPoints.RIGHT_LEVEL_2));
    addSequential(new YeetOffSequence());
    //For without Yeet
    //addSequential(new ElevatorSetpoint(NumberConstants.ELEVATOR_LOW_HATCH_POSITION, NumberConstants.ELEVATOR_MAX_SPEED, 0.5, 1));
    addSequential(new SetXY(FieldPoints.RIGHT_OFF_PLATFORM));
    addSequential(new DriveToGoal(FieldPoints.RIGHT_CLOSE_ROCKET, 5, 0.9, false));
    // addSequential(new TurnToGoal (FieldPoints.RIGHT_ROCKET, 10, 0.9));
    
    addParallel(new SetTrayCommand(false));
    addSequential(new DriveTurn(0.8, 1, true));
    addSequential(new DriveDistance(40, 0, 0.4, 3, true));  
  }
}
