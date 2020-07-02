
package Team4450.Robot20C;

import static Team4450.Robot20C.Constants.*;

import Team4450.Lib.CameraFeed;
import Team4450.Lib.JoyStick;
import Team4450.Lib.LaunchPad;
import Team4450.Lib.MonitorBattery;
import Team4450.Lib.MonitorCompressor;
import Team4450.Lib.MonitorPDP;
import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Lib.JoyStick.JoyStickButtonIDs;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
	private final DriveBase driveBase;
	private final Pickup	pickup;

	// Joy sticks. 3 Joy sticks use RobotLib JoyStick class for some of its extra features. 
	// Specify trigger for monitoring to cause JoyStick event monitoring to not start. We will 
	// use WpiLib button handling instead of RobotLib event monitoring.
	// Launch pad monitoring uses wpilib Joystick class.
	
	private JoyStick	leftStick = new JoyStick(new Joystick(LEFT_STICK), "Left Stick", JoyStickButtonIDs.TRIGGER);
	private JoyStick	rightStick = new JoyStick(new Joystick(RIGHT_STICK), "Right  Stick", JoyStickButtonIDs.TRIGGER);
	private JoyStick	utilityStick = new JoyStick(new Joystick(UTILITY_STICK), "Utility Stick", JoyStickButtonIDs.TRIGGER);
	private Joystick	launchPad = new Joystick(LAUNCH_PAD);	//new LaunchPad(new Joystick(LAUNCH_PAD));

	private AnalogInput	pressureSensor = new AnalogInput(PRESSURE_SENSOR);
	  
	private PowerDistributionPanel	pdp = new PowerDistributionPanel();

	private Compressor	compressor = new Compressor(COMPRESSOR);	// Compressor class represents the PCM.

	private NavX		navx;

	private Thread      		monitorBatteryThread, monitorPDPThread;
	private MonitorCompressor	monitorCompressorThread;
	private CameraFeed			cameraFeed;

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

		// Add navx as a Sendable. Updates the Gyro indicator automatically.
 		
		SmartDashboard.putData("Gyro2", navx);

		// Invert driving joy stick Y axis so + values mean forward.
	  
		leftStick.invertY(true);
		rightStick.invertY(true);
		
		utilityStick.deadZoneY(.25);
		utilityStick.deadZoneX(.25);

		// Create subsystems prior to button mapping.
		
		driveBase = new DriveBase();
		pickup = new Pickup();

		// Configure the button bindings
		configureButtonBindings();
	  
		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the joy sticks to drive the robot. We pass in function
		//  references so the command can read the sticks directly as DoubleProviders.
	  
		driveBase.setDefaultCommand(new DriveCommand(driveBase, () -> leftStick.GetY(), () -> rightStick.GetY()));

   		// Start the battery, compressor, PDP and camera feed monitoring Tasks.

   		monitorBatteryThread = MonitorBattery.getInstance();
   		monitorBatteryThread.start();

   		monitorCompressorThread = MonitorCompressor.getInstance(pressureSensor);
   		monitorCompressorThread.setDelay(1.0);
   		monitorCompressorThread.SetLowPressureAlarm(50);
   		monitorCompressorThread.start();
   		
   		monitorPDPThread = MonitorPDP.getInstance(pdp);
   		monitorPDPThread.start();

		// Start camera server thread using our class for usb cameras.
    
		cameraFeed = CameraFeed.getInstance(); 
		cameraFeed.start();
 		
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
	  
		// Left stick buttons.
		new JoystickButton(leftStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TRIGGER.value)
        	.whenPressed(new ShiftGearsCommand(driveBase));
	  
		// Right stick buttons.
		
		// Utility stick buttons.
		new JoystickButton(utilityStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TOP_BACK.value)
        	.whenPressed(new PickupCommand(pickup));
		
		// Launch pad buttons.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_RED.value)
    	.whenPressed(new InstantCommand(driveBase::resetEncoders, driveBase));
	}

	/**
	 * Use this to pass the autonomous command(s) to the main {@link Robot} class.
	 * @return The command to run in autonomous
	 */
	public Command getAutonomousCommand() 
	{
		Util.consoleLog();
	  
		// An ExampleCommand to be run in autonomous.
		
		return new TestAutoCommand(driveBase);
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