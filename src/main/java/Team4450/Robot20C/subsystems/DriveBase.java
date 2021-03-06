
package Team4450.Robot20C.subsystems;

import static Team4450.Robot20C.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import Team4450.Lib.SRXMagneticEncoderRelative.DistanceUnit;

import Team4450.Robot20C.RobotContainer;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase 
{
	private WPI_TalonSRX		LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon;
	
	private DifferentialDrive	robotDrive;

	private DifferentialDriveOdometry	odometer;
  
	// SRX magnetic encoder plugged into a CAN Talon.
	public SRXMagneticEncoderRelative	leftEncoder, rightEncoder;

	private ValveDA				highLowValve = new ValveDA(HIGHLOW_VALVE);

	private boolean				talonBrakeMode, lowSpeed, highSpeed;
	
	private double				cumulativeLeftCount = 0, cumulativeRightCount = 0;
	private double				lastLeftCount = 0, lastRightCount = 0;
	
	private SlewRateLimiter		leftLimiter = new SlewRateLimiter(0.5);
	private SlewRateLimiter		rightLimiter = new SlewRateLimiter(0.5);

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

   		// Configure starting motor safety. This runs a timer between updates of the
		// robotDrive motor power with the set() method. If the timer expires because
		// of no input, the assumption would be that something has done wrong and the
		// code is no longer feeding the robotDrive with speed commands and so the
		// robot could be in an uncontrolled state. So the watchdog turns off the
		// motors until a new input is delivered by the set() method. The problem is
		// with command based scheme, the drive command is executed once per scheduler
		// run and if there are many commands or long running commands, the scheduler
		// may well not make it back to executing the drive command before the timer
		// expires. Experimentally determined 1 sec timer allows enough time for our
		// commands to complete and scheduler returns to the drive command. 1 sec is
		// a long time out but hopefully the robot cannot go too far off the reservation
		// in one second if some problem prevents new set() calls. Conceivably a command
		// that loops in execute would cause the whole scheduler based scheme to stop
		// and so the drive command would not be run and the robot would drive at last
		// power setting until the watchdog shuts the robotDrive down.
		// One other note: as the length of the command list during a scheduler run
		// lengthens or the commands take too much time, the rate at which the Drive
		// command feeds the drive base will slow down and could lead to a lack of
		// driving response. Using the Drive command as the default command of the
		// DriveBase means the joy sticks will be fed to the motors ONCE per scheduler
		// run. Technically the scheduler runs each time RobotPeriodic() is called which
		// is supposed to be every .02 sec. However, the actual time between runs is
		// the total time of all commands executed and then to the next .02 sec call
		// to RobotPeriodic(). Note that the FIRST lower level code also runs a watchdog
		// on each execution of the .02 sec loop and will raise a warning if your
		// code takes more than .02 sec to complete. It may be hard to stay under
		// that time. When it trips, the watchdog will print to the console some somewhat
		// useful information to help determine where the time is being used. This 
		// watchdog timeout cannot be set or turned off.
   		
   		robotDrive.stopMotor();
   		robotDrive.setSafetyEnabled(false);	// Will be enabled by the Drive command.
   		robotDrive.setExpiration(1.0);
	
		// Always start in low gear with braking enabled.
   		
   		SetCANTalonBrakeMode(true);
		
		lowSpeed();
		
		// Create an odometer object to track the robot's movements.
		
		odometer = new DifferentialDriveOdometry(RobotContainer.navx.getTotalYaw2d());
		
		// Set robot initial position. This is normally set in an auto routine that
		// starts a match at a particular location and angle. If there is no auto
		// defining an initial position, then pose tracking is difficult because it
		// has to start from a known position. If you start in teleop, you would need
		// to define a fixed starting location or perhaps compute it depending on which
		// driver station location and always start in the same place relative to the
		// station.
		
		resetOdometer(new Pose2d(INITIAL_X, INITIAL_Y, new Rotation2d()), INITIAL_HEADING);
	}
	
	// This method will be called once per scheduler run by the scheduler.
	@Override
	public void periodic() 
	{
		// Update the odometer tracking robot position on the field. We have to track the
		// cumulative encoder counts since at any time we can reset the encoders to facilitate
		// driving functions like auto drive, alt driving mode and more. Odometer wants counts
		// as total since start of match or last odometer reset.		
		
		double left = leftEncoder.getDistance(DistanceUnit.Meters);
		double right = rightEncoder.getDistance(DistanceUnit.Meters);
		
		cumulativeLeftCount += left - lastLeftCount;
		cumulativeRightCount += right - lastRightCount;
		
		lastLeftCount = left;
		lastRightCount = right;
		
		odometer.update(RobotContainer.navx.getTotalYaw2d(), cumulativeLeftCount, cumulativeRightCount);
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
	 * Should be called every scheduler run by the Drive command.
	 * @param leftSpeed Left power setting -1.0 to +1.0.
	 * @param rightSpeed RIght power setting -1.0 to +1.0.
	 * @param squaredInputs True reduces sensitivity at low speeds.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs)
	{
		robotDrive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
		
		//Util.consoleLog("l=%.2f m=%.2f  r=%.2f m=%.2f", leftSpeed, LRCanTalon.get(), rightSpeed, RRCanTalon.get());
	}
	
	/**
	 * Tank drive function. Passes left/right speed values to the robot drive.
	 * Should be called every scheduler run by the Drive command. Uses SlewRateLimiter
	 * filter to modulate inputs to smooth out power delivery. Testing did not show any
	 * advantage over squared inputs but will keep this routine for now.
	 * @param leftSpeed Left power setting -1.0 to +1.0.
	 * @param rightSpeed RIght power setting -1.0 to +1.0.
	 */
	public void tankDriveLimited(double leftSpeed, double rightSpeed)
	{
		robotDrive.tankDrive(leftLimiter.calculate(leftSpeed), rightLimiter.calculate(rightSpeed), false);
		
		//Util.consoleLog("l=%.2f m=%.2f  r=%.2f m=%.2f", leftSpeed, LRCanTalon.get(), rightSpeed, RRCanTalon.get());
	}
	
	/**
	 * Curvature drive function. Drives at set speed with set curve.
	 * @param speed Power setting -1.0 to +1.0.
	 * @param curve Rotation rate -1.0 to +1.0. Clockwise us +.
	 * @param quickTurn True causes quick turn (turn in place).
	 */
	public void curvatureDrive(double speed, double curve, boolean quickTurn)
	{
		robotDrive.curvatureDrive(speed, curve, quickTurn);
	}

	// Initialize and Log status indication from CANTalon. If we see an exception
	// or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	private static void InitializeCANTalon(WPI_TalonSRX talon)
	{
		Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		talon.clearStickyFaults(0); //0ms means no blocking.
	}
	  
	/**
	 * Set neutral behavior of drive CAN Talons.
	 * @param brakeMode True = brake mode, false = coast mode.
	 */
	public void SetCANTalonBrakeMode(boolean brakeMode)
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
	 * Toggles drive CAN Talon braking mode.
	 */
	public void toggleCANTalonBrakeMode()
	{
		SetCANTalonBrakeMode(!talonBrakeMode);
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
		
	private void updateDS()
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
	 * Reset the drive wheel encoders to zero.
	 */
	public void resetEncoders()
	{
		Util.consoleLog();
		
		leftEncoder.reset();
		rightEncoder.reset();
	}
	
	/**
	 * Reset the drive wheel encoders to zero with time delay. There can be a significant delay
	 * between calling for encoder reset and the encoder returning zero. Sometimes this does not
	 * matter and other times it can really mess things up if you reset encoder but at the time
	 * you next read the encoder for a measurement (like in autonomous programs) the encoder has
	 * not yet been reset and returns the previous count. This method resets and delays 112ms
	 * which testing seemed to show would cause the next read of the reset encoder to return
	 * zero.
	 */
	public void resetEncodersWithDelay()
	{
		Util.consoleLog();
		
		Util.consoleLog("at encoder reset le=%d  re=%d", leftEncoder.get(), rightEncoder.get());
		
		// Set encoders to update every 100ms.
		rightEncoder.setStatusFramePeriod(100);
		leftEncoder.setStatusFramePeriod(100);

		// Reset encoders with 112ms delay before proceeding.
		int rightError = rightEncoder.reset(2);
		int leftError = leftEncoder.reset(110);
		
		Util.consoleLog("after reset le=%d  re=%d  sl=%d  sr=%d", leftEncoder.get(), rightEncoder.get(),
						leftError, rightError);	
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
		Util.consoleLog("%s", enabled);
		
		robotDrive.setSafetyEnabled(enabled);
	}
	
	/**
	 * Get current pose from odometer. Pose distances in meters.
	 * Pose X is distance along the long side of the field from your driver
	 * station wall. Y is distance along the short side of the field starting
	 * on the left. Angle is referenced from zero as pointing directly at the
	 * opposition driver station wall, + is left, - is right of that zero
	 * alignment.
	 * @return Current pose.
	 */
	public Pose2d getOdometerPose()
	{
		return odometer.getPoseMeters();
	}
	
	/**
	 * Reset odometer to new position and cumulative angle. Pose x,y distances
	 * in meters, as described in getOdometerPose() doc.
	 * @param pose New starting pose.
	 * @param heading Heading of robot (cumulative angle).
	 */
	public void resetOdometer(Pose2d pose, double heading)
	{
		odometer.resetPosition(pose, Rotation2d.fromDegrees(heading));
		
		cumulativeLeftCount = 0;
		cumulativeRightCount = 0;
	}
	
	/** 
	 * Average left and right encoder counts to see how far robot has moved.
	 * @return Average of left & right encoder counts.
	 */
	public int getAvgEncoder()
	{
		return (leftEncoder.get() + rightEncoder.get()) / 2;
	}
	
	/** 
	 * Left encoder counts.
	 * @return Left encoder counts.
	 */
	public int getLeftEncoder()
	{
		return leftEncoder.get();
	}
	
	/** 
	 * Left encoder counts.
	 * @return Left encoder counts.
	 */
	public int getRightEncoder()
	{
		return rightEncoder.get();
	}
}
