package frc.robot.commands.auto;

import frc.robot.NumberConstants;
import frc.robot.Robot;
import frc.robot.loops.VisionLoop;
import edu.wpi.first.wpilibj.command.Command;
/**
 * 
 * @author RickHansenRobotics
 * @since 2019-02-03
 *
 */
public class CameraTrackDrive extends Command {

	private double timeOut; //Sets the integer for the timeout
	private double xVal;    //Sets the integer for xVal as the average of X Cordinates
	private double prevXVal = 0; 
	private double degree;//Sets the integer for the degree converted from xVal inputed into PixelsToDegrees Equation
	private boolean hasChanged = true;
  private boolean started = false;
  
  VisionLoop visionLoop; 
		
    public CameraTrackDrive() {
    	this(2.0);
    }
    
    public CameraTrackDrive(double timeOut) {
      visionLoop = VisionLoop.getInstance(); 
    	this.timeOut = timeOut;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	hasChanged = true;
    	started = false;
    	prevXVal = 0;
    	if(timeOut>0)
    		setTimeout(timeOut);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(true) {

	    	xVal = visionLoop.avg();

        degree = visionLoop.pixelToDegree(xVal); 
      }



    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(timeOut==-1) {
    		return false;
    	} else {
    		return isTimedOut();
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.runLeftDrive(0);
    	Robot.drive.runRightDrive(0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.runLeftDrive(0);
    	Robot.drive.runRightDrive(0);
    }
}