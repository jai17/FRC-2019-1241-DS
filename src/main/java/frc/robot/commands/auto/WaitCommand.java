package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * @author RickHansen Robotics
 * @since 02/06/2019
 */
public class WaitCommand extends Command {

	private double wait;
    
	public WaitCommand(double wait) {
        this.wait = wait;
    }

    protected void initialize() {
    	setTimeout(wait);
    }

    protected void execute() {
    }

    protected boolean isFinished() {
        return isTimedOut();
    }

    protected void end() {
        System.out.println("WC_END: " + Robot.drive.getAngle());
    }

    protected void interrupted() {
    }
}