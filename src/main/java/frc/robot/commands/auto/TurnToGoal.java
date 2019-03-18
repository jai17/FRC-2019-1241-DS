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

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FieldPositioning;
import frc.robot.util.Point;

/**
 *	TurnToGoal
 *	@author Neil Balaskandarajah
 *	Turn to face a point on the field, with PID regulated outputs based on your current field-relative pose.
 */
public class TurnToGoal extends Command {
	private Point goalPoint; //goal point to drive to
	private double goalYaw; //goal angle to maintain; updated based on robot field position
	private long initTime; //initial time
	private double epsilon; //tolerance in degrees for final heading
	private double topSpeed; //top speed limit for PID
	private int printCounter; //counter for prints
	// private PrintWriter pw; //print to .csv
	private double goalAngle;

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
		printCounter = 0;
		
		driveLoop = DrivetrainLoop.getInstance(); 
		drive = Drivetrain.getInstance(); 
    }

   	//called in autonInit() ONCE
    protected void initialize() {
    	Point robotPoint = Robot.drive.getXYPoint();
        goalYaw = FieldPositioning.calcGoalYaw(robotPoint, goalPoint); //setpoint
        
        //debug
        // try {
		// 	pw = new PrintWriter(new File("/home/lvuser/tests/turnToGoal.csv"));
	    //     pw.println("time,goalAngle,currentAngle,sideVel,isAngle,isSpeed");
		// } catch (FileNotFoundException e) {
		// 	System.out.println(this.toString() + ": ERROR: could not create file!");
		// }
        
        System.out.println(this.toString() +": INITIALIZING: "+ goalYaw +" "+ goalPoint.getxPos() +" "+ goalPoint.getyPos());
        initTime = System.currentTimeMillis();
        
        //get change in angle from yaw
        goalAngle = Robot.drive.getAngle() + Math.min(goalYaw - Robot.drive.getYaw(), goalYaw + Robot.drive.getYaw());
		
		driveLoop.setAnglePID(goalAngle);
		driveLoop.setRelativePID(true);
		driveLoop.setTolerancePID(epsilon);
		driveLoop.setTopSpeed(topSpeed);
		driveLoop.setPIDType(false);
		driveLoop.setDriveState(DriveControlState.POINT_FOLLOWING);
	}

    //repeatedly called when required to execute
    protected void execute() {
		
    	SmartDashboard.putNumber("goalYaw", goalAngle);
    	
    	//print to file
    	// pw.println((System.currentTimeMillis()-initTime) +","+ goalYaw +","+ Robot.drive.getYaw() +","+ Robot.drive.getLeftVelocityInchesPerSec()
    	// 		+","+ ((Robot.drive.getYaw() >= (goalYaw - epsilon) && Robot.drive.getYaw() <= (goalYaw + epsilon)) 
    	// 		+","+ (Math.abs(Robot.drive.getLeftVelocityInchesPerSec()) <= 20)));
    	
    	//print to console
    	if(printCounter % 20 == 0) {
    		System.out.println(this.toString() +": RUNNING: "+ goalYaw + " @ " + Robot.drive.getYaw());
    	}
    	printCounter++;
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
    	// pw.close();
		Point robotPoint = Robot.drive.getXYPoint();
		drive.runLeftDrive(0);
		drive.runRightDrive(0);
		driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
    	System.out.println(this.toString() +": FINISHED: turned to "+ goalYaw +" @ "+ Robot.drive.getYaw());
    	System.out.println(robotPoint.getxPos() +" "+ robotPoint.getyPos());
    	System.out.println();
    }

    //called when interrupted
    protected void interrupted() {
    	// pw.close();
    	System.out.println(this.toString() +": interrupted");
    }
}