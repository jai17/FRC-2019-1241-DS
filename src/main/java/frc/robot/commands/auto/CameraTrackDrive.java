package frc.robot.commands.auto;

import frc.robot.Robot;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.DrivetrainLoop.DriveControlState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
/**
 * 
 * @author RickHansenRobotics
 * @since 2019-02-03
 *
 */
public class CameraTrackDrive extends Command {

  private double timeOut; //Sets the integer for the timeout
  private double distance; 
  private double speed; 
  private double tolerance; 
	private double xVal;    //Sets the integer for xVal as the average of X Cordinates
	private double degree;//Sets the integer for the degree converted from xVal inputed into PixelsToDegrees Equation
  
  Vision visionLoop; 
  DrivetrainLoop driveLoop; 
  Drivetrain drive; 

  Timer timer;
	private boolean timerStarted = false;
    
    public CameraTrackDrive(double distance, double speed, double timeOut, double tolerance) {
      visionLoop = Vision.getInstance(); 
      driveLoop = DrivetrainLoop.getInstance(); 
      drive = Drivetrain.getInstance(); 
      this.distance = distance; 
      this.speed = speed; 
      this.timeOut = timeOut;
      this.tolerance = tolerance; 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(timeOut>0)
        setTimeout(timeOut);
      drive.resetEncoders();
      timer = new Timer(); 
      driveLoop.setDriveState(DriveControlState.TRACKING);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
	    	xVal = visionLoop.avg();
        degree = visionLoop.pixelToDegree(xVal); 
        driveLoop.setDistancePID(distance);
        driveLoop.setSpeedPID(speed);
        driveLoop.setAnglePID(drive.getAngle() - degree);
        driveLoop.setTolerancePID(tolerance);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(Math.abs(distance - Robot.drive.getAvgPos()) <= tolerance){
          if(!timerStarted){
            timer.start();
            timerStarted = true;
          }if(timer.get() > 0.25){
            Robot.logger.logd("TrackDriveCommand", "Got to setpoint: " + distance);
            return true;
          }
        }else{
        timer.stop();
        timer.reset();
        timerStarted = false;
      }
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	drive.runLeftDrive(0);
    	drive.runRightDrive(0);
      drive.resetDrivePID();
      drive.resetGyroPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	drive.runLeftDrive(0);
      drive.runRightDrive(0);
      driveLoop.setDriveState(DriveControlState.OPEN_LOOP);
    }
}