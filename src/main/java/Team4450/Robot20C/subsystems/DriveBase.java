
package Team4450.Robot20C.subsystems;

import static Team4450.Robot20C.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase 
{
	private WPI_TalonSRX		LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon;
	
	private DifferentialDrive	robotDrive;
	  
	// SRX magnetic encoder plugged into a CAN Talon.
	public SRXMagneticEncoderRelative	leftEncoder, rightEncoder;

	private ValveDA					highLowValve = new ValveDA(HIGHLOW_VALVE);

	private boolean					talonBrakeMode, lowSpeed, highSpeed;
	
	/**
	 * Creates a new DriveBase Subsystem.
	 */
	public DriveBase()
	{
		Util.consoleLog();
		
		// Create the drive Talons.
		LFCanTalon = new WPI_TalonSRX(LF_TALON);
		LRCanTalon = new WPI_TalonSRX(LR_TALON);
		RFCanTalon = new WPI_TalonSRX(RF_TALON);	
		RRCanTalon = new WPI_TalonSRX(RR_TALON);	
		
		// Initialize CAN Talons and write status to log so we can verify
		// all the Talons are connected.
		InitializeCANTalon(LFCanTalon);
		InitializeCANTalon(LRCanTalon);
		InitializeCANTalon(RFCanTalon);
		InitializeCANTalon(RRCanTalon);
		
		// Configure CAN Talons with correct inversions.
		LFCanTalon.setInverted(true);
		LRCanTalon.setInverted(true);
		  
		// These should be true for regular tank. false for 
		// velocity tank.
		RFCanTalon.setInverted(true);
		RRCanTalon.setInverted(true);

		// 2018 post season testing showed Anakin liked this setting, smoothing driving.
		SetCANTalonRampRate(TALON_RAMP_RATE);
		  
		// Configure SRX encoders as needed for measuring velocity and distance. 
		// Wheel diameter is in inches. Adjust for each years robot.
		
		rightEncoder = new SRXMagneticEncoderRelative(RRCanTalon, DRIVE_WHEEL_DIAMETER);
		leftEncoder = new SRXMagneticEncoderRelative(LRCanTalon, DRIVE_WHEEL_DIAMETER);
		  
		leftEncoder.setInverted(true);

		// For 2020 robot, put rear talons into a differential drive object and set the
	    // front talons to follow the rears.
		  
		LFCanTalon.set(ControlMode.Follower, LRCanTalon.getDeviceID());
		RFCanTalon.set(ControlMode.Follower, RRCanTalon.getDeviceID());
		  
		robotDrive = new DifferentialDrive(LRCanTalon, RRCanTalon);

   		// Configure starting motor safety;
   		
   		robotDrive.stopMotor();
   		robotDrive.setSafetyEnabled(false);
   		robotDrive.setExpiration(0.1);
	
		// Always start in low gear.
		
		lowSpeed();
	}
	
	@Override
	public void periodic() 
	{
		// This method will be called once per scheduler run
	}
	
	/**
	 * Stops all motors.
	 */
	public void stop()
	{
		Util.consoleLog();
		
		robotDrive.stopMotor();
	}
	
	/**
	 * Tank drive function. Passes left/right speed values to the robot drive.
	 * @param leftSpeed Left power setting -1.0 to +1.0.
	 * @param rightSpeed RIght power setting -1.0 to +1.0.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed)
	{
		robotDrive.tankDrive(leftSpeed, rightSpeed);
	}

	// Initialize and Log status indication from CANTalon. If we see an exception
	// or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	private static void InitializeCANTalon(WPI_TalonSRX talon)
	{
		Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		talon.clearStickyFaults(0); //0ms means no blocking.
	}
	  
	// Set neutral behavior of drive CAN Talons. True = brake mode, false = coast mode.

	private void SetCANTalonBrakeMode(boolean brakeMode)
	{
		Util.consoleLog("brakes on=%b", brakeMode);
		  
		SmartDashboard.putBoolean("Brakes", brakeMode);

		talonBrakeMode = brakeMode;
		  
		NeutralMode newMode;
		  
		if (brakeMode) 
			newMode = NeutralMode.Brake;
		else 
			newMode = NeutralMode.Coast;
		  
		 LFCanTalon.setNeutralMode(newMode);
		 LRCanTalon.setNeutralMode(newMode);
		 RFCanTalon.setNeutralMode(newMode);
		 RRCanTalon.setNeutralMode(newMode);
	}
	  
	/**
	 * Returns drive Talon brake mode.
	 * @return True if Talons set to brake, false if coast.
	 */
	public boolean isBrakeMode()
	{
		return talonBrakeMode;
	}  
	  
	/**
	 * Set CAN Talon voltage ramp rate.
	 * @param seconds Number of seconds from zero to full output.
	 * zero disables.
	 */
	public void SetCANTalonRampRate(double seconds)
	{
		Util.consoleLog("%.2f", seconds);
		  
		LFCanTalon.configOpenloopRamp(seconds, 0);
		LRCanTalon.configOpenloopRamp(seconds, 0);
		RFCanTalon.configOpenloopRamp(seconds, 0);
		RRCanTalon.configOpenloopRamp(seconds, 0);
	}
	  
	// Return voltage and current draw for each CAN Talon.
	  
	public String GetCANTalonStatus()
	{
		return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				  LFCanTalon.getMotorOutputVoltage(), LFCanTalon.getStatorCurrent(),
				  LRCanTalon.getMotorOutputVoltage(), LRCanTalon.getStatorCurrent(),
				  RFCanTalon.getMotorOutputVoltage(), RFCanTalon.getStatorCurrent(),
				  RRCanTalon.getMotorOutputVoltage(), RRCanTalon.getStatorCurrent()
				  );
	}
		
	protected void updateDS()
	{
		Util.consoleLog("low=%b, high=%b", lowSpeed, highSpeed);
			
		SmartDashboard.putBoolean("Low", lowSpeed);
		SmartDashboard.putBoolean("High", highSpeed);
	}

	/**
	 * Set gear boxes into low speed. Pushes the dog ring to the inside.
	 */
	public void lowSpeed()
	{
		Util.consoleLog();

		highSpeed = false;

		highLowValve.SetA();
			
		lowSpeed = true;
			
		updateDS();
	}

	/**
	 * Set gear boxes into high speed. Pushes the dog ring to the outside.
	 */
	public void highSpeed()
	{
		Util.consoleLog();

		lowSpeed = false;
			
		highLowValve.SetB();
			
		highSpeed = true;
			
		updateDS();
	}

	/**
	 * Return low speed state.
	 * @return True if low speed.
	 */
	public boolean isLowSpeed()
	{
		return lowSpeed;
	}
		
	/**
	 * Return high speed state.
	 * @return True if high speed.
	 */
	public boolean isHighSpeed()
	{
		return highSpeed;
	}
	
	/**
	 * Reset the drive wheel encoders.
	 */
	public void resetEncoders()
	{
		Util.consoleLog();
		
		leftEncoder.reset();
		rightEncoder.reset();
	}
	
	/**
	 * Return right side motor power.
	 * @return Power level -1.0 to +1.0.
	 */
	public double getRightPower()
	{
		return RRCanTalon.get();
	}
	
	/**
	 * Return left side motor power.
	 * @return Power level -1.0 to +1.0.
	 */
	public double getLeftPower()
	{
		return LRCanTalon.get();
	}
	
	/**
	 * Enable/disable drive base motor safety watchdog.
	 * @param enabled True to enable watchdog, false to disable.
	 */
	public void setMotorSafety(boolean enabled)
	{
		robotDrive.setSafetyEnabled(enabled);
	}
}
