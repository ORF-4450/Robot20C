
package Team4450.Robot20C;

import static Team4450.Robot20C.Constants.*;

import Team4450.Lib.CameraFeed;
import Team4450.Lib.JoyStick;
import Team4450.Lib.LaunchPad;
import Team4450.Lib.NavX;
import Team4450.Lib.Util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import Team4450.Robot20C.commands.DriveCommand;
import Team4450.Robot20C.commands.PickupCommand;
import Team4450.Robot20C.commands.ShiftGearsCommand;
import Team4450.Robot20C.commands.TestAutoCommand;
import Team4450.Robot20C.subsystems.DriveBase;
import Team4450.Robot20C.subsystems.Pickup;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
	// The robot's subsystems and commands are defined here...
	private final DriveBase driveBase = new DriveBase();
	private final Pickup	pickup = new Pickup();

	private final TestAutoCommand autoCommand = new TestAutoCommand(driveBase);

	public JoyStick		leftStick = new JoyStick(new Joystick(LEFT_STICK), "Left Stick");
	public JoyStick		rightStick = new JoyStick(new Joystick(RIGHT_STICK), "Right  Stick");
	public JoyStick		utilityStick = new JoyStick(new Joystick(UTILITY_STICK), "Utility Stick");
	public LaunchPad	launchPad = new LaunchPad(new Joystick(LAUNCH_PAD));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() throws Exception
  {
	  Util.consoleLog();
      
	  getMatchInformation();
	  
	  // Read properties file from RoboRio "disk".
      
	  robotProperties = Util.readProperties();
      
	  // Is this the competition or clone robot?
   		
	  if (robotProperties.getProperty("RobotId").equals("comp"))
		  isComp = true;
	  else
		  isClone = true;
 		
	  // Set compressor enabled switch on dashboard from properties file.
	  // Later code will read that setting from the dashboard and turn 
	  // compressor on or off in response to dashboard setting.
 		
	  SmartDashboard.putBoolean("CompressorEnabled", Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault")));

	  // Reset PDB & PCM sticky faults.
    
	  pdp.clearStickyFaults();
	  compressor.clearAllPCMStickyFaults();

	  // Create NavX object here since must done before CameraFeed is created (don't remember why).
	  // Navx calibrates at power on and must complete before robot moves. Takes 12 seconds.

	  navx = NavX.getInstance(NavX.PortType.SPI);

	  // Add navx as a Sendable. Updates the Gyro indicator automatically when 
	  // SmartDashboard.updateValues() is called elsewhere.
 		
	  SmartDashboard.putData("Gyro2", navx);

	  // Invert driving joy stick Y axis so + values mean f.
	  
	  leftStick.invertY(true);
	  rightStick.invertY(true);
		
	  utilityStick.deadZoneY(.25);
	  utilityStick.deadZoneX(.25);

	  // Configure the button bindings
	  configureButtonBindings();
	  
	  // Set the default drive command. This command will be scheduled automatically to run
	  // every loop and so handle the joy sticks to drive the robot.
	  
	  driveBase.setDefaultCommand(new DriveCommand(driveBase, () -> leftStick.GetY(), () -> rightStick.GetY()));

	  // Start camera server thread using our class for usb cameras.
    
	  cameraThread = CameraFeed.getInstance(); 
	  cameraThread.start();
 		
	  // Log info about NavX.
	  
	  navx.dumpValuesToNetworkTables();
 		
	  if (navx.isConnected())
		  Util.consoleLog("NavX version=%s", navx.getAHRS().getFirmwareVersion());
	  else
	  {
		  Exception e = new Exception("NavX is NOT connected!");
		  Util.logException(e);
	  }
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
	  Util.consoleLog();
	  
	  new JoystickButton(leftStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TRIGGER.value)
        .whenPressed(new ShiftGearsCommand(driveBase));
	  
	  new JoystickButton(utilityStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TOP_BACK.value)
        .whenPressed(new PickupCommand(pickup));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
	  Util.consoleLog();
	  
	  // An ExampleCommand will run in autonomous
	  return autoCommand;
  }
  
  // Get and log information about the current match from the FMS or DS.
  
  public void getMatchInformation()
  {
  	  alliance = ds.getAlliance();
  	  location = ds.getLocation();
	  eventName = ds.getEventName();
	  matchNumber = ds.getMatchNumber();
	  gameMessage = ds.getGameSpecificMessage();
    
      Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, ds.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
  }
  
}
