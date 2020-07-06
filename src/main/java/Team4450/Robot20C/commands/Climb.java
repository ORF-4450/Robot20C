
package Team4450.Robot20C.commands;

import java.util.function.DoubleSupplier;

import static Team4450.Robot20C.Constants.*;
import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Robot20C.subsystems.Climber;
import Team4450.Robot20C.subsystems.DriveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Climbing command that feeds % power to the Climber.
 */
public class Climb extends CommandBase 
{
  private final Climber climber;
  
  private final DoubleSupplier	climbPower;

  /**
   * Creates a new Drive command.
   *
   * @param subsystem The subsystem used by this command.
   * @param leftSpeed The speed as % power -1.0 to +1.0.
   * @param rightSpeed The speed as % power -1.0 to +1.0.
   */
  public Climb(Climber subsystem, DoubleSupplier climbPower) 
  {
	  Util.consoleLog();
	  
	  climber = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  
	  addRequirements(this.climber);
	  
	  this.climbPower = climbPower;
  }

  /**
   *  Called when the command is initially scheduled.
   *  NOTE: This command is set as the default for the climber. That
   *  means it runs as long as no other command that uses the climber
   *  runs. If another command runs, this command will be interrupted 
   *  and then rescheduled when that other command is finished. That 
   *  reschedule means initialize() is called again. So it is important 
   *  to realize this command does not "exist" for the entire run of teleop.
   *  It comes and goes when it is preempted by another command. 
   *  All commands work like this.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled. Passes
   * the climb power value provided by whatever double provider was passed
   * in the constructor to the climber setWinchPower() function. The provider is
   * typically the utility stick Y deflection value but can be any double provider.
   */
  @Override
  public void execute() 
  {

	  climber.setWinchPower(climbPower.getAsDouble());
  }

  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  climber.stop();
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
