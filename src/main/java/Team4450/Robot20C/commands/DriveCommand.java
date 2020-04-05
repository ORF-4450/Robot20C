
package Team4450.Robot20C.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot20C.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Driving command that feeds target speeds (% power) to the DriveBase.
 */
public class DriveCommand extends CommandBase 
{
  private final DriveBase driveBase;
  
  private final DoubleSupplier	leftSpeed, rightSpeed;

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param leftSpeed The speed as % power.
   * @param rightSpeed The speed as % power.
   */
  public DriveCommand(DriveBase subsystem, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) 
  {
	  Util.consoleLog();
	  
	  driveBase = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  addRequirements(this.driveBase);
	  
	  this.leftSpeed = leftSpeed;
	  this.rightSpeed = rightSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
	  driveBase.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  driveBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
	  return false;
  }
}
