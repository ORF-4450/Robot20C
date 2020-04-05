package Team4450.Robot20C.commands;

import Team4450.Lib.Util;
import Team4450.Robot20C.subsystems.Pickup;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that toggles Pickup operation.
 */
public class PickupCommand extends CommandBase 
{
  private final Pickup pickup;

  /**
   * Creates a new PickupCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PickupCommand(Pickup subsystem) 
  {
	  Util.consoleLog();
	  
	  pickup = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  addRequirements(this.pickup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
	  
	  if (pickup.isExtended())
		  pickup.retract();
	  else
		  pickup.extend();
  }

  // Returns true when the command should end. This means one execution.
  @Override
  public boolean isFinished() 
  {
	  return true;
  }
}
