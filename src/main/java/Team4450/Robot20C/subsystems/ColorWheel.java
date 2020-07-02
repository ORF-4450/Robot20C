package Team4450.Robot20C.subsystems;

import static Team4450.Robot20C.Constants.*;

import Team4450.Lib.Util;

/**
 * Color Wheel subsystem.
 */
public class ColorWheel
{

	public ColorWheel()
	{
		Util.consoleLog();
		
		Util.consoleLog("ColorWheel created!");
	}
	
	/**
	 * Convert single letter color code to full color word.
	 * @param gameColor
	 * @return
	 */
	public static String convertGameColor(String gameColor)
	{
		String color = "";
		
		if (gameColor == null) return color;
		
		if (gameColor.length() == 0) return color;
		
		switch (gameColor.charAt(0))
		{
			case 'B' :
				color = "BLUE";
				break;
				
			case 'G' :
				color = "GREEN";
				break;
				
			case 'R' :
				color = "RED";
				break;
				
			case 'Y' :
				color = "YELLOW";
				break;
				
			default :
				color = "";
				break;
	  }

		return color;
	}

}
