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
import frc.robot.Robot;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Point;

import edu.wpi.first.wpilibj.command.Command;

/**
 *	DriveToGoal
 *	@author Neil Balaskandarajah
 *	Drive to a point on the field, with PID regulated outputs based on your current field-relative pose.
 */
public class DriveToGoal extends Command {
	private Point goalPoint; //goal point to drive to
	private double totalDist; //total distance for PID to drive
	private double initTotalDist; //initial total distance (1st hypotenuse of first current point - now)
	private double goalYaw; //goal angle to maintain; updated based on robot field position
	private long initTime; //initial time
	private double epsilon; //tolerance; radius of circle about goal point which you can exist in
	private double topSpeed; //top speed limit for PID
	private PrintWriter pw; //print to .csv
	private int printCounter; //counter to save memory
	private Point robotPoint; //point for robot position
	private double currentSetpoint; //current distance you are tracking right now

	DrivetrainLoop driveLoop;
	Drivetrain drive;
	
	/* COMMAND
	 * goal - point to drive to
	 * epsilon - radius of circle around point which point can lie in for command to finish
	 * topSpeed - top speed for PID value to pseudo-circumvent the inherent assumption of instantaneous acceleration
	 *		- may be changed to an acceleration profile at beginning (end behavious of PID adheres to robot constraints closely enough)
	 */
    public DriveToGoal(Point goal, double epsilon, double topSpeed) {
        requires(Robot.drive);
        goalPoint = goal;
        this.epsilon = epsilon;
        this.topSpeed = topSpeed;
		printCounter = 0;
		
		driveLoop = Robot.driveLoop;
		drive = Robot.drive;
    }

    /* INITIALIZE
     * 1) calculate initial total distance, total distance to drive
     * 2) calculate current heading for point (should be zero as we use DT... paths)
     * 3) calculate total time for D term in PID
     * 4) store initial distance value for PID (based on encoders, driving and turning from previous increases encoder values)
     */
    protected void initialize() {
        totalDist = Point.calcDistance(drive.getXYPoint(), goalPoint);
        initTotalDist = totalDist;
        
        //debug
        try {
			pw = new PrintWriter(new File("/home/lvuser/tests/driveToGoal.csv"));
	        pw.println("time,x,y,avgVel,goalAngle,totalDist");
		} catch (FileNotFoundException e) {
			System.out.println(this.toString() +": ERROR: could not create file!");
		}
        
        goalYaw = frc.robot.util.FieldPositioning.calcGoalYaw(Robot.drive.getXYPoint(), goalPoint);
        initTime = System.currentTimeMillis();
        System.out.println("\n" + this.toString() +": INITIALIZING: "+ goalPoint.getxPos() +","+ goalPoint.getyPos()); //dbg
        System.out.println("initTotalDist:" + initTotalDist +" initYaw:" + goalYaw + " @ " + Robot.drive.getYaw());
        System.out.println("x y goalYaw currentYaw");
    }

    /*EXECUTE
     * 1) live update distance (up to a point)
     * 2) live update angle (up to a point further from start)
     * 3) PID for distance and angle
     * dbg - print data to file
     */
    protected void execute() {
    	robotPoint = Robot.drive.getXYPoint();
    	
    	//(1) live update distance
    	totalDist = Point.calcDistance(Robot.drive.getXYPoint(), goalPoint); //live update distance when far from point
    	
    	//(2) when close to point, maintain current heading
    	if(totalDist >= (initTotalDist*0.15)) { //if point distance is greater than 15% of total distance (not too close to point)
    		goalYaw = FieldPositioning.calcGoalYaw(robotPoint, goalPoint); //live update goal angle when far from point
    		currentSetpoint = Robot.drive.getAveragePos() + totalDist;
    	}
    	
    	//drive setpoint is *ahead* of where you are
    	Robot.drive.regulatedDrivePID(currentSetpoint, goalYaw, epsilon, (System.currentTimeMillis() - initTime), topSpeed);
    		
    	//print to .csv file
    	pw.println((System.currentTimeMillis() - initTime) +","+ robotPoint.getxPos() +","+ robotPoint.getyPos() +","+
    		Robot.drive.getAverageVelInchesPerSec() +","+ goalYaw +","+ totalDist);
    	
    	//print to console
    	if(printCounter % 10 == 0) {
    		System.out.println(robotPoint.getxPos() +" "+ robotPoint.getyPos() +" "+ goalYaw +" "+ Robot.drive.getYaw());
    	}
    	printCounter++;
    }

    /* FINISHED
     * drive until within circle around point and moving slowly
     */
    protected boolean isFinished() {
        
      if (Point.isWithinBounds(goalPoint, Robot.drive.getXYPoint(), epsilon)) { //within radius of circle
        	return true;
        	
        } else {
        	return false;
        }
    }

    //run when finished
    protected void end() {
    	pw.close();
    	Point endPoint = Robot.drive.getXYPoint();
    	System.out.println(this.toString() +": FINISHED: drove to "+ goalPoint.getxPos() +","+ goalPoint.getyPos() 	
    		+" @ "+ endPoint.getxPos() +","+ endPoint.getyPos() +","+ Robot.drive.getYaw());
    	System.out.println();
    }

    //don't interrupt me 
    protected void interrupted() {
    	pw.close();
    	System.out.println(this.toString() +": interrupted");
    }
}