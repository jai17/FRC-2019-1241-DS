/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FieldPositioning;
import frc.robot.util.Logger;
import frc.robot.util.Point;

/**
 *	TurnToGoal
 *	@author Neil Balaskandarajah
 *	Turn to face a point on the field, with PID regulated outputs based on your current field-relative pose.
 */
public class TurnToGoal extends Command {
	private Point goalPoint; //goal point to drive to
	private double goalYaw; //goal angle to maintain; updated based on robot field position
	private double epsilon; //tolerance in degrees for final heading
	private double topSpeed; //top speed limit for PID
	private double goalAngle; //goal angle for the robot to turn to

	private Logger logger = Logger.getInstance(); //debugging

	DrivetrainLoop driveLoop; 
	Drivetrain drive; 
	
	/**
	 * Turn towards a goal point at a regulated speed with a tolerance.
	 * @param goal - goal point to turn to.
	 * @param epsilon - tolerance (+/-) to which the robot must turn.
	 * @param topSpeed - maximum magnitude of left/right side of robot between -1 and 1.
	 */
    public TurnToGoal(Point goal, double epsilon, double topSpeed) {
        requires(Robot.drive);
        goalPoint = goal;
        this.epsilon = epsilon;
        this.topSpeed = topSpeed;
		
		driveLoop = DrivetrainLoop.getInstance(); 
		drive = Drivetrain.getInstance(); 
	}

   	//called when command starts ONCE
    protected void initialize() {
    	Point robotPoint = Robot.drive.getXYPoint(); //update robot point 
        goalYaw = FieldPositioning.calcGoalYaw(robotPoint, goalPoint); //yaw value
        
        //get change in angle from yaw
        goalAngle = Robot.drive.getAngle() + Math.min(goalYaw - Robot.drive.getYaw(), goalYaw + Robot.drive.getYaw());
		
		driveLoop.setAnglePID(goalAngle); //delta angle is setpoint
		driveLoop.setRelativePID(false); //set to use angle, not yaw
		driveLoop.setTolerancePID(epsilon); //set tolerance for the turn in degrees
		driveLoop.setTopSpeed(topSpeed); //set top speed to clamp turn to
		driveLoop.setPIDType(false); // use turnPID
		driveLoop.setDriveState(DriveControlState.POINT_FOLLOWING); //update state 

		logger.logd("TurnToGoal_init(): ", "Goal Angle: " + Double.toString(goalAngle));
	}

    //repeatedly called when required to execute
    protected void execute() {
		
    }

    //end condition for command
    protected boolean isFinished() {
    	if((Robot.drive.getAngle() >= (goalAngle - epsilon) && Robot.drive.getAngle() <= (goalAngle + epsilon)) && //if heading is in range
    			(Math.abs(drive.getRightOutput()) <= 0.25)) { //and low oscillation on PID; getting side vel bc average vel when turning is 0 
			return true;    		
    	} else {
    		return false;
    	}
    }

    //called at the end
    protected void end() {
		Point robotPoint = Robot.drive.getXYPoint();

		//stop drive
		drive.runLeftDrive(0);
		drive.runRightDrive(0);
		driveLoop.setDriveState(DriveControlState.OPEN_LOOP);

		//debug
    	logger.logd("TurnToGoal_end(): ", "Current (X,Y) = " + robotPoint.toString() + 
					 " Goal (X,Y) = " + goalPoint.toString() + 
					 " Drive Yaw = " + Double.toString(drive.getYaw()));
    }

    //called when interrupted
    protected void interrupted() {
		logger.logd("TurnToGoal_interrupted(): ", "Interrupted");
    }
}