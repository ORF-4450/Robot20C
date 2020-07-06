package Team4450.Robot20C.commands;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import static Team4450.Robot20C.Constants.*;
import Team4450.Robot20C.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAuto extends CommandBase
{
	private final DriveBase driveBase;

	/**
	 * Creates a new TestAuto autonomous command.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public TestAuto(DriveBase subsystem) 
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
		
	  	LCD.printLine(LCD_1, "Mode: Auto - TestAutoCommand - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				ds.isFMSAttached(), gameMessage);
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 */
	@Override
	public void execute() 
	{
		Util.consoleLog();
		
		//driveBase.tankDrive(.50, .50);
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
	 *  Returns true when the command should end.
	 */
	@Override
	public boolean isFinished() 
	{
		return false;	// false makes this command run until auto mode ends.
	}
}