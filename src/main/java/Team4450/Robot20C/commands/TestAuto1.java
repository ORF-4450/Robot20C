package Team4450.Robot20C.commands;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;

import static Team4450.Robot20C.Constants.*;
import Team4450.Robot20C.RobotContainer;
import Team4450.Robot20C.subsystems.DriveBase;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto1 extends CommandBase
{
	private final DriveBase driveBase;
	
	private SequentialCommandGroup	commands = null;
	private Command					command = null;

	/**
	 * Creates a new TestAuto1 autonomous command.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public TestAuto1(DriveBase subsystem) 
	{
		Util.consoleLog();
		
		driveBase = subsystem;
			  
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(this.driveBase);
	}
	
	/**
	 * Called when the command is initially scheduled. (non-Javadoc)
	 * @see edu.wpi.first.wpilibj2.command.Command#initialize()
	 */
	@Override
	public void initialize() 
	{
		Util.consoleLog();
		
		driveBase.setMotorSafety(false);  // Turn off watchdog.
		
	  	LCD.printLine(LCD_1, "Mode: Auto - TestAuto1 - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				ds.isFMSAttached(), gameMessage);
		
		// Reset wheel encoders.	  	
	  	driveBase.resetEncodersWithDelay();
	  	
	  	// Set NavX yaw tracking to 0.
	  	RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed during the match.
		RobotContainer.navx.setHeading(0);
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(0);
			
		// Set Talon ramp rate for smooth acceleration from stop. Determine by observation.
		driveBase.SetCANTalonRampRate(1.0);
			
		// Reset odometry tracking.
		driveBase.resetOdometer(new Pose2d(0.0, 0.0, RobotContainer.navx.getTotalYaw2d()));
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();
		
		// First action is to drive forward 2000 encoder counts and stop with brakes on.
		
		command = new AutoDrive(driveBase, .25, 2000, 
								AutoDrive.StopMotors.stop,
								AutoDrive.Brakes.on,
								AutoDrive.Pid.off,
								AutoDrive.Heading.angle);
		
		commands.addCommands(command);
		
		// Launch autonomous command sequence.
		
		commands.schedule();
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 *  In this model, this command just idles while the Command Group we
	 *  created runs on its own executing the steps (commands) of this Auto
	 *  program.
	 */
	@Override
	public void execute() 
	{
	}
	
	/**
	 *  Called when the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();
	}
	
	/**
	 *  Returns true when the command should end. That should be when
	 *  all the commands in the command list have finished.
	 */
	@Override
	public boolean isFinished() 
	{
		return commands.isFinished();
	}
}

