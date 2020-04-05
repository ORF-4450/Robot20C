package Team4450.Robot20C.commands;

import Team4450.Lib.Util;
import Team4450.Robot20C.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that toggles gear selection on the DriveBase.
 */
public class ShiftGearsCommand extends CommandBase 
{
  private final DriveBase driveBase;

  /**
   * Creates a new ShiftGearsCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShiftGearsCommand(DriveBase subsystem) 
  {
	  Util.consoleLog();
	  
	  driveBase = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  addRequirements(this.driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
	  
	  if (driveBase.isLowSpeed())
		  driveBase.highSpeed();
	  else
		  driveBase.lowSpeed();
  }

  // Returns true when the command should end. This means one execution.
  @Override
  public boolean isFinished() 
  {
	  return true;
  }
}
