  /*----------------------------------------------------------------------------*/
  /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
  /* Open Source Software - may be modified and shared by FRC teams. The code   */
  /* must be accompanied by the FIRST BSD license file in the root directory of */
  /* the project.                                                               */
  /*----------------------------------------------------------------------------*/

  package frc.robot.commands.hatch;

  import edu.wpi.first.wpilibj.command.Command;
  import frc.robot.NumberConstants;
  import frc.robot.Robot;
  import frc.robot.loops.HatchLoop;
  import frc.robot.loops.HatchLoop.HatchControlState;
  import frc.robot.subsystems.Hatch;
  import frc.robot.util.ToggleBoolean;

  public class HatchCommand extends Command {
    HatchLoop hatchLoop;
    Hatch hatch;

    boolean openLoop = true;
    ToggleBoolean toggle;

    public HatchCommand() {
      hatchLoop = HatchLoop.getInstance();
      hatch = Hatch.getInstance();
      requires(Robot.hatch);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      // have to hold left trigger for hatch
      /*
      if (Robot.m_oi.getToolLeftTrigger()) { 

        if (Robot.m_oi.getToolLeftBumper()) { // outtake
          hatchLoop.setIsIntaking(false);
          hatchLoop.setRollerSpeed(0.78);
          hatchLoop.setHatchState(HatchControlState.MOTION_MAGIC);

        } else if (Robot.m_oi.getToolRightBumper()) { //intake
          hatchLoop.setIsIntaking(true);
          hatchLoop.setRollerSpeed(0.78);
          hatchLoop.setHatchState(HatchControlState.MOTION_MAGIC);

        } else { // button pressed, no hatch panel
          hatchLoop.setRollerSpeed(0);
          hatchLoop.setHatchState(HatchControlState.MOTION_MAGIC);
        }

        // open loop
        if ((Robot.m_oi.getToolLeftY() > 0.5 || Robot.m_oi.getToolLeftY() < -0.5 && !openLoop)) {
          openLoop = true;
        }

        // not open loop
        if ((Robot.m_oi.getToolRightTrigger()) && openLoop) {
          openLoop = false;
        } 

        if (!openLoop){
          if (Robot.m_oi.getToolRightTrigger()) { //hatch intaking height
            hatchLoop.setMotionMagic(NumberConstants.HATCH_INTAKING_ANGLE, NumberConstants.HATCH_MAX_SPEED, 0.1);
            hatchLoop.setHatchState(HatchControlState.MOTION_MAGIC);
            openLoop = false; 
            
          } else { //hatch rest height
            hatchLoop.setMotionMagic(NumberConstants.HATCH_REST_ANGLE, NumberConstants.HATCH_MAX_SPEED, 0.1);
            hatchLoop.setHatchState(HatchControlState.MOTION_MAGIC);
            openLoop = false;
          }

        }

        if (openLoop) {
          if (Robot.m_oi.getToolLeftY() > 0.5) { // move down
            hatchLoop.setPivotSpeed(-1);
            hatchLoop.setHatchState(HatchControlState.OPEN_LOOP);

          } else if (Robot.m_oi.getToolLeftY() < -0.5) { // move up
            hatchLoop.setPivotSpeed(1);
            hatchLoop.setHatchState(HatchControlState.OPEN_LOOP);

          } else { // don't move
            hatchLoop.setPivotSpeed(0);
            hatchLoop.setHatchState(HatchControlState.OPEN_LOOP);
          }
        }
*/
      }
    

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
  }
