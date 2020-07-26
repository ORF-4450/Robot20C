
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import Team4450.Robot20C.commands.Climb;
import Team4450.Robot20C.commands.Drive;
import Team4450.Robot20C.commands.NotifierCommand2;
import Team4450.Robot20C.commands.ShiftGears;
import Team4450.Robot20C.commands.TestAuto;
import Team4450.Robot20C.commands.TurnWheelCounting;
import Team4450.Robot20C.commands.TurnWheelToColor;
import Team4450.Robot20C.subsystems.Climber;
import Team4450.Robot20C.subsystems.ColorWheel;
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
	private final DriveBase 	driveBase;
	private final Pickup		pickup;
	private final ColorWheel	colorWheel;
	private final Climber		climber;
	private final Drive			driveCommand;
	
	private final TurnWheelCounting	turnWheelCounting;
	private final InstantCommand	cancelTurnWheelCounting;

	// Joy sticks. 3 Joy sticks use RobotLib JoyStick class for some of its extra features. 
	// Specify trigger for monitoring to cause JoyStick event monitoring to not start. We will 
	// use WpiLib button handling instead of RobotLib event monitoring.
	// Launch pad monitoring uses regular wpilib Joystick class.
	
	private JoyStick	leftStick = new JoyStick(new Joystick(LEFT_STICK), "Left Stick", JoyStickButtonIDs.TRIGGER);
	private JoyStick	rightStick = new JoyStick(new Joystick(RIGHT_STICK), "Right  Stick", JoyStickButtonIDs.TRIGGER);
	public JoyStick		utilityStick = new JoyStick(new Joystick(UTILITY_STICK), "Utility Stick", JoyStickButtonIDs.TRIGGER);
	private Joystick	launchPad = new Joystick(LAUNCH_PAD);	//new LaunchPad(new Joystick(LAUNCH_PAD));

	private AnalogInput	pressureSensor = new AnalogInput(PRESSURE_SENSOR);
	  
	private PowerDistributionPanel	pdp = new PowerDistributionPanel();

	private Compressor	compressor = new Compressor(COMPRESSOR);	// Compressor class represents the PCM.
	
	public static NavX	navx;

	private Thread      		monitorBatteryThread, monitorPDPThread;
	private MonitorCompressor	monitorCompressorThread;
	private CameraFeed			cameraFeed;

	// List of autonomous programs.
	private enum AutoProgram
	{
		NoProgram,
		TestAuto
	}

	private static SendableChooser<AutoProgram>	autoChooser;

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
    
		resetFaults();

		// Create NavX object here since must done before CameraFeed is created (don't remember why).
		// Navx calibrates at power on and must complete before robot moves. Takes 12 seconds.

		navx = NavX.getInstance(NavX.PortType.SPI);

		// Add navx as a Sendable. Updates the Gyro indicator automatically.
 		
		SmartDashboard.putData("Gyro2", navx);

		// Invert driving joy stick Y axis so + values mean forward.
	  
		leftStick.invertY(true);
		rightStick.invertY(true);  
		
		// Invert utility stick so pulling back is + which means go up.
		
		utilityStick.invertX(true);

		utilityStick.deadZoneY(.25);
		utilityStick.deadZoneX(.25);

		// Create subsystems prior to button mapping.
		
		driveBase = new DriveBase();
		pickup = new Pickup();
		colorWheel = new ColorWheel();
		climber = new Climber(() -> utilityStick.GetX());
		
		// Create any persistent commands.
		
		turnWheelCounting = new TurnWheelCounting(colorWheel);
		cancelTurnWheelCounting = new InstantCommand(turnWheelCounting::cancel);
		
		// Set the default climb command. This command will be scheduled automatically to run
		// every teleop period and so use the utility joy stick to control the climber winch.
		// We pass in function reference so the command can read the stick directly as a
		// DoubleProvider.
		
		climber.setDefaultCommand(new Climb(climber, () -> utilityStick.GetY()));
	  
		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the joy sticks to drive the robot. We pass in function
		// references so the command can read the sticks directly as DoubleProviders.
	  
		driveBase.setDefaultCommand(driveCommand = new Drive(driveBase, () -> leftStick.GetY(), () -> rightStick.GetY()));

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
		
		setAutoChoices();

		// Configure the button bindings
		
		configureButtonBindings();
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
	  
		// ------- Left stick buttons --------------
		
		// Shift gears.
		new JoystickButton(leftStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TRIGGER.value)
        	.whenPressed(new ShiftGears(driveBase));
  
		// ------- Right stick buttons -------------

		// Toggle alternate driving mode.
		new JoystickButton(rightStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TRIGGER.value)
    	.whenPressed(new InstantCommand(driveCommand::toggleAlternateDrivingMode));
  
		// -------- Utility stick buttons ----------
		
		// Toggle extend Pickup.
		// So we show 3 ways to control the pickup. A regular command that toggles pickup state,
		// an instant command that calls a method on Pickup class that toggles state and finally
		// our special notifier variant that runs the Pickup class toggle method in a separate
		// thread. So we show all 3 methods as illustration but the reason we tried 3 methods is
		// that the pickup retraction action takes almost 1 second (due apparently to some big
		// overhead in disabling the electric eye interrupt) and triggers the global and drivebase
		// watchdogs. Threading does not as the toggle method is not run on the scheduler thread.
		
		new JoystickButton(utilityStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TOP_BACK.value)
        	//.whenPressed(new PickupDeploy(pickup));		
			//.whenPressed(new InstantCommand(pickup::toggleDeploy, pickup));
			.whenPressed(new NotifierCommand2(pickup::toggleDeploy, 0.0, pickup));
		
		// -------- Launch pad buttons -------------
		
		// Because the launch pad buttons are wired backwards, we use whenReleased to 
		// react on button press instead of whenPressed.
		
		// Reset encoders.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_RED.value)
    		.whenReleased(new InstantCommand(driveBase::resetEncoders));
		
		// Toggle color wheel motor on/off. Also stops the count and to-color commands but
		// you have to click twice to stop those commands.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_BLUE.value)
    		.whenReleased(new InstantCommand(colorWheel::toggleWheel, colorWheel));
		
		// Start command to turn color wheel specified number of turns.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_BLUE_RIGHT.value)
    		.whenReleased(new ConditionalCommand(turnWheelCounting, cancelTurnWheelCounting, turnWheelCounting::isScheduled));
	
		// Start command to turn color wheel to target color sent by FMS.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_YELLOW.value)
    		.whenReleased(new TurnWheelToColor(colorWheel));
		
		// Toggle climber brake. Note we don't supply climber as subsystem on this command
		// to get around a quirk in how the scheduler works...because on the toggle brake
		// function we will schedule the Traverse command and we want traverse to suspend
		// the climb command while traverse is active. Specifying a subsystem here fools the
		// scheduler and it runs Traverse and Climb commands at the same time, which is not
		// what we want (no climb while brake engaged!).
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_RED_RIGHT.value)
			.whenReleased(new InstantCommand(climber::toggleBrake));
	
		// Toggle drive CAN Talon brake mode. We need to capture both sides of the rocker switch
		// to get a toggle on either position of the rocker.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.ROCKER_LEFT_BACK.value)
			.whenPressed(new InstantCommand(driveBase::toggleCANTalonBrakeMode));
	
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.ROCKER_LEFT_BACK.value)
    		.whenReleased(new InstantCommand(driveBase::toggleCANTalonBrakeMode));
		
		// Toggle camera feeds. We need to capture both sides of the rocker switch to get a toggle
		// on either position of the rocker.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.ROCKER_LEFT_FRONT.value)
    		.whenPressed(new InstantCommand(cameraFeed::ChangeCamera));

		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.ROCKER_LEFT_FRONT.value)
	    	.whenReleased(new InstantCommand(cameraFeed::ChangeCamera));
	}

	/**
	 * Use this to pass the autonomous command(s) to the main {@link Robot} class.
	 * Determines which auto command from the selection made by the operator on the
	 * DS drop down list of commands.
	 * @return The command to run in autonomous
	 */
	public Command getAutonomousCommand() 
	{
		AutoProgram		program = AutoProgram.NoProgram;
		Command			autoCommand = null;
		
		Util.consoleLog();

		try
		{
			program = autoChooser.getSelected();
		}
		catch (Exception e)	{ Util.logException(e); }
		
		switch (program)
		{
			case NoProgram:
				autoCommand = null;
				break;
				
			case TestAuto:
				autoCommand = new TestAuto(driveBase);
				break;
		}

		// The command to be run in autonomous.
		
		return autoCommand;
	}
  
	/**
	 *  Get and log information about the current match from the FMS or DS.
	 */
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
		
	// Configure SendableChooser (drop down list) with auto program choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private static void setAutoChoices()
	{
		Util.consoleLog();
		
		autoChooser = new SendableChooser<AutoProgram>();
		
		SendableRegistry.add(autoChooser, "Auto Program");
		autoChooser.setDefaultOption("No Program", AutoProgram.NoProgram);
		autoChooser.addOption("Test Auto Program", AutoProgram.TestAuto);		
				
		SmartDashboard.putData(autoChooser);
	}

	/**
	 * Reset sticky faults in PDP and PCM and turn compressor on/off as
	 * set by switch on DS.
	 */
	public void resetFaults()
	{
		// This code turns off the automatic compressor management if requested by DS. Putting this
		// in this function is a convenience since it is called at each mode change.
		compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));

		pdp.clearStickyFaults();
		compressor.clearAllPCMStickyFaults();
	}
}
