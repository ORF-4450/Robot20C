
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
	public static String		PROGRAM_NAME = "RAC20C-07.02.20-1";

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false;
	    	
	public static DriverStation.Alliance	alliance;
	public static int                       location, matchNumber;
	public static String					eventName, gameMessage;
	    
	// Drive motor controller port assignments.
	public static final int		LF_TALON = 1, LR_TALON = 2, RF_TALON = 3, RR_TALON = 4;
	
	// Other motor controller port assignments
	public static final int		PICKUP_TALON = 6, BELT_TALON = 7, WINCH_FRONT_VICTOR = 8, WINCH_BACK_VICTOR = 9;
	public static final int		HOOK_VICTOR = 10, COLOR_WHEEL_VICTOR = 11;
	
	// Joystick port assignments.
	public static final int		LEFT_STICK = 0, RIGHT_STICK = 1, UTILITY_STICK = 2, LAUNCH_PAD = 3;

	// Pneumatic valve controller port assignments.
	public static final int		COMPRESSOR = 0;
	public static final int		HIGHLOW_VALVE = 0;			// 0-1 
	public static final int		PICKUP_VALVE = 2; 			// 2-3
	public static final int		CLIMBER_BRAKE_VALVE = 4;	// 4-5

	// Digital Input port assignments.
	public static final int		WINCH_SWITCH = 0, WINCH_ENCODER = 1, BALL_EYE = 3;
	  
	// Analog Input port assignments.
	public static final int		PRESSURE_SENSOR = 0;

	public static final DriverStation	ds = DriverStation.getInstance();

	public static final double	TALON_RAMP_RATE = 0.5, DRIVE_WHEEL_DIAMETER = 6.125;
	
	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	// Robot, TestAutoCommand.
	public static final int		LCD_2 = 2;	// DriveCommand.
	public static final int		LCD_3 = 3;	// DriveCommand.
	public static final int		LCD_7 = 7;	// DriveCommand
	
}
