

package Team4450.Robot20C.commands;

import java.util.function.DoubleSupplier;

import static Team4450.Robot20C.Constants.*;
import Team4450.Lib.LCD;
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
   * @param leftSpeed The speed as % power -1.0 to +1.0.
   * @param rightSpeed The speed as % power -1.0 to +1.0.
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

  /**
   *  Called when the command is initially scheduled.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
	  
	  driveBase.setMotorSafety(true); 	// Turn on watchdog.
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled. Passes
   * the left/right speed values provided by whatever double provider was passed
   * to the constructor to the drive base tank drive function. The providers are
   * typically the joystick Y deflection values but can be any double provider.
   */
  @Override
  public void execute() 
  {
	  double leftY = leftSpeed.getAsDouble(), rightY = rightSpeed.getAsDouble();
	  
	  LCD.printLine(LCD_3, "leftY=%.3f (%.3f)  rightY=%.3f (%.3f)", leftY, 
				 driveBase.getLeftPower(), rightY, driveBase.getRightPower());
	  
	  driveBase.tankDrive(leftY, rightY);
  }

  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  driveBase.stop();
  }

  /**
   *  Returns true when the command should end. Returning false means it never ends.
   */
  @Override
  public boolean isFinished() 
  {
	  return false;
  }
}
