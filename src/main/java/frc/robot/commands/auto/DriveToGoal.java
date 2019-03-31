/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;

import frc.robot.util.FieldPositioning;
import frc.robot.util.Logger;
import frc.robot.Robot;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Point;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.command.Command;

/**
 * DriveToGoal
 * 
 * @author Neil Balaskandarajah Drive to a point on the field, with PID
 *         regulated outputs based on your current field-relative pose.
 */
public class DriveToGoal extends Command {
	private Point goalPoint; // goal point to drive to
	private double totalDist; // total distance for PID to drive
	private double initTotalDist; // initial total distance (1st hypotenuse of first current point - now)
	private double goalYaw; // goal angle to maintain; updated based on robot field position
	private double epsilon; // tolerance; radius of circle about goal point which you can exist in
	private double topSpeed; // top speed limit for PID
	private Point robotPoint; // point for robot position
	private double currentSetpoint; // current distance you are tracking right now
	private boolean reverse; // whether the robot is driving in reverse or not
	private boolean highPID; // whether robot drives in high gear or not

	DrivetrainLoop driveLoop;
	Drivetrain drive;

	Logger logger = Logger.getInstance();

	public DriveToGoal(Point goal, double epsilon, double topSpeed, boolean reverse) {
		requires(Robot.drive);
		this.goalPoint = goal;
		this.epsilon = epsilon;
		this.topSpeed = topSpeed;
		this.reverse = reverse;
		this.highPID = false;

		driveLoop = DrivetrainLoop.getInstance();
		drive = Drivetrain.getInstance();
	}

	public DriveToGoal(Point goal, double epsilon, double topSpeed, boolean reverse, boolean highPID) {
		requires(Robot.drive);
		this.goalPoint = goal;
		this.epsilon = epsilon;
		this.topSpeed = topSpeed;
		this.reverse = reverse;
		this.highPID = highPID;

		driveLoop = DrivetrainLoop.getInstance();
		drive = Drivetrain.getInstance();
	}

	/*
	 * INITIALIZE 1) calculate initial total distance, total distance to drive 2)
	 * calculate current heading for point (should be zero as we use DT... paths) 3)
	 * calculate total time for D term in PID 4) store initial distance value for
	 * PID (based on encoders, driving and turning from previous increases encoder
	 * values)
	 */
	protected void initialize() {
		// calculate distances from the goal point
		totalDist = Point.calcDistance(drive.getXYPoint(), goalPoint);
		initTotalDist = totalDist;

		double goalYawTemp = FieldPositioning.calcGoalYaw(Robot.drive.getXYPoint(), goalPoint);

		// if driving in reverse
		if (reverse) {
			// change setpoint to reverse
			if (Math.signum(goalYawTemp) == 1) {
				goalYawTemp -= 180;
			} else {
				goalYawTemp += 180;
			}

			// make distance setpoints negative
			totalDist = -totalDist;
			initTotalDist = -initTotalDist;
		}

		if (goalYawTemp >= 0) {
			goalYaw = Robot.drive.getAngle()
					+ Math.min(goalYawTemp - Robot.drive.getYaw(), goalYawTemp + Robot.drive.getYaw());
		} else {
			goalYaw = Robot.drive.getAngle()
					+ Math.max(goalYawTemp - Robot.drive.getYaw(), goalYawTemp + Robot.drive.getYaw());
		}
		// cartesian left is 0, yaw left is 90

		// calculate drive setpoint
		currentSetpoint = drive.getAveragePos() + totalDist;

		// update state machine
		driveLoop.setDistancePID(currentSetpoint); // set distance setpoint
		driveLoop.setRelativePID(false); // set to use yaw
		driveLoop.setHighPID(highPID); // set to gear
		driveLoop.setAnglePID(goalYaw); // set angle septoint
		driveLoop.setTolerancePID(epsilon); // set tolerance for point radius
		driveLoop.setTopSpeed(topSpeed); // set top speed to regulate to
		driveLoop.setPIDType(true); // true for drive
		driveLoop.setDriveState(DriveControlState.POINT_FOLLOWING); // set the proper state

		// cancel ramp rate
		drive.setLeftrampRate(0);
		drive.setRightrampRate(0);

		logger.logd("DriveToGoal_init(): ",
				"Distance = " + Double.toString(currentSetpoint) + " Yaw = " + Double.toString(goalYaw));
	}

	/*
	 * EXECUTE 1) live update distance (up to a point) 2) live update angle (up to a
	 * point further from start) 3) PID for distance and angle
	 */
	protected void execute() {
		double goalYawTemp = 0;
		// update robot position
		robotPoint = drive.getXYPoint();

		// (1) live update distance
		totalDist = Point.calcDistance(robotPoint, goalPoint); // live update distance when far from point
		if (reverse) {
			totalDist = -totalDist;
		}

		// (2) when close to point, maintain current heading, update point
		if (totalDist >= (initTotalDist * 0.1)) { // if point distance is greater than 15% of total distance (not too
													// close to point)
			// goalYaw = FieldPositioning.calcGoalYaw(robotPoint, goalPoint); //live update
			// goal angle when far from point

			goalYawTemp = FieldPositioning.calcGoalYaw(Robot.drive.getXYPoint(), goalPoint);

			if (reverse) {
				if (Math.signum(goalYawTemp) == 1) {
					goalYawTemp -= 180;
				} else {
					goalYawTemp += 180;
				}
			}

			if (goalYawTemp >= 0) {
				goalYaw = Robot.drive.getAngle()
						+ Math.min(goalYawTemp - Robot.drive.getYaw(), goalYawTemp + Robot.drive.getYaw());
			} else {
				goalYaw = Robot.drive.getAngle()
						+ Math.max(goalYawTemp - Robot.drive.getYaw(), goalYawTemp + Robot.drive.getYaw());
			}
			
			currentSetpoint = drive.getAveragePos() + totalDist;
		}

		// flip yaw setpoints if driving in reverse

		// dbg
		logger.logd("DriveToGoal_execute(): ", "Distance = " + Double.toString(currentSetpoint) + ", Yaw = "
				+ Double.toString(goalYaw) + ", GoalYawTemp" + Double.toString(goalYawTemp));

		// update distance and yaw setpoints
		driveLoop.setDistancePID(currentSetpoint);
		driveLoop.setAnglePID(goalYaw);
	}

	/*
	 * FINISHED drive until within circle around point
	 */
	protected boolean isFinished() {
		if (Point.isWithinBounds(goalPoint, drive.getXYPoint(), epsilon)) { // within radius of circle

			return true;
		} else {
			return false;
		}
	}

	// run when finished
	protected void end() {
		logger.logd("DriveToGoal_end(): ", "Current (X,Y) = " + robotPoint.toString() + " Goal (X,Y) = "
				+ goalPoint.toString() + " Drive Encoder = " + Double.toString(drive.getAveragePos()));

		drive.runRightDrive(0);
		drive.runLeftDrive(0);
		driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
	}

	// don't interrupt me
	protected void interrupted() {
		logger.logd("DriveToGoal_interrupted(): ", "Interrupted");
	}
}