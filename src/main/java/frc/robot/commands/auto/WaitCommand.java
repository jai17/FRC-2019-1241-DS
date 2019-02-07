package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;

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
    }

    protected void interrupted() {
    }
}