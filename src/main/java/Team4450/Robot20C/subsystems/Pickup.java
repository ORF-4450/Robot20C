package Team4450.Robot20C.subsystems;

import static Team4450.Robot20C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import Team4450.Lib.Util;

import edu.wpi.first.wpilibj.InterruptHandlerFunction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase
{
	public static WPI_TalonSRX	pickupTalon;
	
	private boolean				extended = false, pickupRunning = false;
	
	public Pickup ()
	{
		Util.consoleLog();

		pickupTalon = new WPI_TalonSRX(PICKUP_TALON);

		// Configure interrupt handler for the ballEye optical ball detector.
		
		ballEye.requestInterrupts(new InterruptHandler());
		
		// Listen for a falling edge interrupt.
		
		ballEye.setUpSourceEdge(false, true);

		retract();
		
		Util.consoleLog("Pickup created!");
	}

	protected void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Pickup", pickupRunning);
		SmartDashboard.putBoolean("PickupExtended", extended);
	}
	
	public void extend()
	{
		Util.consoleLog();
		
		pickupValve.SetA();
		
		extended = true;
		
		start(.50);
		
		updateDS();
	}
	
	public void retract()
	{
		Util.consoleLog();
		
		stop();

		pickupValve.SetB();
		
		extended = false;
		
		updateDS();
	}
	
	public void start(double power)
	{
		Util.consoleLog("%.2f", power);
		
		pickupTalon.set(power);
		
		pickupRunning = true;
		
		ballEye.enableInterrupts();
		
		updateDS();
	}

	public void stop()
	{
		Util.consoleLog();
		
		pickupTalon.stopMotor();
	
		pickupRunning = false;
		
		ballEye.disableInterrupts();
	
		updateDS();
	}
	
	public boolean isExtended()
	{
		return extended;
	}
	
	public boolean isRunning()
	{
		return pickupRunning;
	}
		
	// Interrupt handler object to handle detection of ball by the ball Eye
	// optical sensor. The sensor generates a hardware interrupt when the 
	// eye is triggered and the fired method is called when interrupt occurs.
	// Keep the length of the code in that method short as no new interrupts
	// will be reported until fired method ends.
	private class InterruptHandler extends InterruptHandlerFunction<Object> 
	{
	     @Override
	     public void interruptFired(int interruptAssertedMask, Object param) 
	     {
	    	 Util.consoleLog("ball  interrupt");
	    	 
	    	 //channel.intakeBall();
	    	 
	    	 //Channel channel = (Channel) param;
	    	 //channel.intakeBall();
	     }
	     
//		 public Channel overridableParamter()
//	     {
//			return channel;
//	     }
	}
}
