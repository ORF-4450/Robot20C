
package Team4450.Robot20C;

import java.util.Properties;

import Team4450.Lib.CameraFeed;
import Team4450.Lib.NavX;
import Team4450.Lib.ValveDA;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "RAC20C-04.03.20";

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false;
	  
	//public static RobotState		currentRobotState = RobotState.boot, lastRobotState = RobotState.boot;
	    	
	public static DriverStation.Alliance	alliance;
	public static int                       location, matchNumber;
	public static String					eventName, gameMessage;
	    
	//Thread               	monitorBatteryThread, monitorPDPThread;
	//MonitorCompressor		monitorCompressorThread;
	public static CameraFeed	cameraThread;

	// Drive motor Can Talon port assignments.
	public static final int		DBLF = 1, DBLR = 2, DBRF = 3, DBRR = 4;
	
	public static final int		PICKUP_TALON = 6;
	
	// Joystick port assignments.
	public static final int		LEFT_STICK = 0, RIGHT_STICK = 1, UTILITY_STICK = 2, LAUNCH_PAD = 4;

	public final static ValveDA	highLowValve = new ValveDA(0);			// For gearbox.
	public final static ValveDA	pickupValve = new ValveDA(2);			// For pickup arm.
	public final static ValveDA	climberBrake = new ValveDA(4);			// For climber brake.
	  
	public static DigitalInput	winchSwitch = new DigitalInput(0);
	public static DigitalInput	ballEye = new DigitalInput(3);
	  
	public final static AnalogInput	pressureSensor = new AnalogInput(0);
	  
	public final static PowerDistributionPanel	pdp = new PowerDistributionPanel();

	public final static Compressor	compressor = new Compressor(0);		// Compressor class represents the PCM.

	public final static DriverStation	ds = DriverStation.getInstance();

	public static NavX			navx;

	public static final double	TALON_RAMP_RATE = 0.5;
}
