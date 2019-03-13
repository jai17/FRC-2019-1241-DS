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
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Point;
import edu.wpi.first.wpilibj.IterativeRobotBase;
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
	private double epsilon; //tolerance; radius of circle about goal point which you can exist in
	private double topSpeed; //top speed limit for PID
	private int printCounter; //counter to save memory
	private Point robotPoint; //point for robot position
	private double currentSetpoint; //current distance you are tracking right now
	private boolean reverse; //whether the robot is driving in reverse or not

	DrivetrainLoop driveLoop;
	Drivetrain drive;
	
	/* COMMAND
	 * goal - point to drive to
	 * epsilon - radius of circle around point which point can lie in for command to finish
	 * topSpeed - top speed for PID value to pseudo-circumvent the inherent assumption of instantaneous acceleration
	 *		- may be changed to an acceleration profile at beginning (end behavious of PID adheres to robot constraints closely enough)
	 */
    public DriveToGoal(Point goal, double epsilon, double topSpeed, boolean reverse) {
        requires(Robot.drive);
        this.goalPoint = goal;
        this.epsilon = epsilon;
		this.topSpeed = topSpeed;
		this.reverse = reverse;
		printCounter = 0;
		
		driveLoop = DrivetrainLoop.getInstance();
		drive = Drivetrain.getInstance();
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
		
        // //debug
        // try {
		// 	pw = new PrintWriter(new File("/home/lvuser/tests/driveToGoal.csv"));
	    //     pw.println("time,x,y,avgVel,goalAngle,totalDist");
		// } catch (FileNotFoundException e) {
		// 	System.out.println(this.toString() +": ERROR: could not create file!");
		// }
        
		goalYaw = FieldPositioning.calcGoalYaw(Robot.drive.getXYPoint(), goalPoint);
		//cartesian left is 0, yaw left is 90
        System.out.println("\n" + this.toString() +": INITIALIZING: "+ goalPoint.getxPos() +","+ goalPoint.getyPos()); //dbg
        System.out.println("initTotalDist:" + initTotalDist +" initYaw:" + (goalYaw) + " @ " + Robot.drive.getYaw());
		System.out.println("x y goalYaw currentYaw");
		
		if (reverse) {
			if (Math.signum(goalYaw) == 1){
				goalYaw -= 180; 
			} else {
				goalYaw += 180;
			}
			totalDist = -totalDist;
		}

		//loops
		driveLoop.setDistancePID(totalDist);
		driveLoop.setAnglePID(goalYaw);
		driveLoop.setTolerancePID(epsilon);
		driveLoop.setTopSpeed(topSpeed);
		driveLoop.setPIDType(true); //true for drive
		driveLoop.setDriveState(DriveControlState.POINT_FOLLOWING);
    }

    /*EXECUTE
     * 1) live update distance (up to a point)
     * 2) live update angle (up to a point further from start)
     * 3) PID for distance and angle
     * dbg - print data to file
     */
    protected void execute() {
    	robotPoint = drive.getXYPoint();
    	
    	//(1) live update distance
    	totalDist = Point.calcDistance(robotPoint, goalPoint); //live update distance when far from point
		if (reverse){
			totalDist = -totalDist; 
		}

    	//(2) when close to point, maintain current heading
    	if(totalDist >= (initTotalDist*0.1)) { //if point distance is greater than 15% of total distance (not too close to point)
    		goalYaw = FieldPositioning.calcGoalYaw(robotPoint, goalPoint); //live update goal angle when far from point
			// if ((goalPoint.getxPos() - robotPoint.getxPos()) != 0) {
			// 	goalYaw = Math.atan((goalPoint.getyPos() - robotPoint.getyPos()) / 
			// 	(goalPoint.getxPos() - robotPoint.getxPos()));
			// } else {
			// 	goalYaw = 0;
			// }
			currentSetpoint = drive.getAveragePos() + totalDist;
		}
		
		if (reverse){
			if (Math.signum(goalYaw) == 1){
				goalYaw -= 180; 
			} else {
				goalYaw += 180;
			}
		} else {
			driveLoop.setAnglePID(goalYaw);
		}
		
		driveLoop.setDistancePID(currentSetpoint);
		//drive setpoint is *ahead* of where you are
    		
    	// //print to .csv file
    	// pw.println((System.currentTimeMillis() - initTime) +","+ robotPoint.getxPos() +","+ robotPoint.getyPos() +","+
    	// 	Robot.drive.getAverageVelInchesPerSec() +","+ goalYaw +","+ totalDist);
    	
    	//print to console
    	if(printCounter % 5 == 0) {
    		System.out.println(goalPoint.getxPos() +" "+ goalPoint.getyPos() +" "+ robotPoint.getxPos() +" "+ robotPoint.getyPos() +" "+ (goalYaw) +" "+ Robot.drive.getYaw());
    	}
    	printCounter++;
    }

    /* FINISHED
     * drive until within circle around point
     */
    protected boolean isFinished() {
        
      if (Point.isWithinBounds(goalPoint, drive.getXYPoint(), epsilon)) { //within radius of circle
        	return true;
        } else {
        	return false;
        }
    }

    //run when finished
    protected void end() {
    	// pw.close();
    	Point endPoint = drive.getXYPoint();
    	System.out.println(this.toString() +": FINISHED: drove to "+ goalPoint.getxPos() +","+ goalPoint.getyPos() 	
    		+" @ "+ endPoint.getxPos() +","+ endPoint.getyPos() +","+ Robot.drive.getYaw());
		System.out.println();
		driveLoop.setLeftDrive(0);
		driveLoop.setRightDrive(0);
		driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
		
    }

    //don't interrupt me 
    protected void interrupted() {
    	// pw.close();
    	System.out.println(this.toString() +": interrupted");
    }
}