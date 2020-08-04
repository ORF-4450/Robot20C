package Team4450.Robot20C.commands;

import Team4450.Lib.LCD;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;

import static Team4450.Robot20C.Constants.*;
import Team4450.Robot20C.RobotContainer;
import Team4450.Robot20C.subsystems.DriveBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoRotate extends CommandBase
{
	private final DriveBase driveBase;

	private double			yaw, elapsedTime = 0, power, target; 
	private double			kP = .02, kI = 0.003, kD = 0.001, kTolerance = 1.0, kSteeringGain = .10;
	private boolean			usePid,	useHeading;
	
	SynchronousPID			pidController = null;
	
	/**
	 * Creates a new AutoRotate command.
	 * 
	 * Auto rotate the specified target angle from where the robot is currently pointing or rotate
	 * to a target heading.
	 * 
	 * @param subsystem The subsystem used by this command.
	 * @param power Max power for rotation. Power is always +.
	 * @param target Target angle (-left, +right) to rotate from robot current direction -180..+180, or 
	 * target heading (0..359) to rotate to from robot current heading. Target heading is always + and cannot be more 
	 * than 180 degrees away from current heading.
	 * @param usePid False for simple rotation, true use PID controller to manage the rotation slowing rotation as
	 * target is reached.
	 * @param useHeading False target is an angle, true target is a heading.
	 * 
	 * Note: This routine is designed for tank drive and the P,I,D,tolerance values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to turn smoothly
	 * and accurately.
	 */
	public AutoRotate(DriveBase subsystem, 
					  double power, 
					  double target, 
					  boolean usePid, 
					  boolean useHeading)
	{
		driveBase = subsystem;
		
		Util.consoleLog("pwr=%.2f  target=%.2f  pid=%b  hdg=%b", power, target, usePid, useHeading);
		
		if (power <= 0) throw new IllegalArgumentException("power must be +");
			  
		this.power = power;
		this.target = target;
		this.usePid = usePid;
		this.useHeading = useHeading;
		
		addRequirements(this.driveBase);
	}
	
	@Override
	public void initialize()
	{
		Util.consoleLog();
		
		// Try to prevent over rotation.
		
		driveBase.SetCANTalonBrakeMode(true);

		// Reset yaw to current robot direction or target heading.
		
		if (useHeading) 
		{
			Util.checkRange(target, 0, 359, "target");
			
			RobotContainer.navx.setTargetHeading(target);
		}
		else
		{
			Util.checkRange(target, 180, "target");
			
			RobotContainer.navx.resetYawWait(1, 500);
		}
		
		if (usePid)
		{
			// Use PID to control power as we turn slowing as we approach target heading.
			
			pidController = new SynchronousPID(kP, kI, kD);
			
			pidController.setOutputRange(-power , power);
			
			if (useHeading)
				pidController.setSetpoint(0);		// We are trying to get the yaw to zero.
			else
				pidController.setSetpoint(target);	// We are trying to get to the target yaw.
			
			// The PID class needs delta time between calls to calculate the I term.
			
			Util.getElaspedTime();
		}
		else if (useHeading)			// Simple turn, full power until target heading reached.
		{
			yaw = RobotContainer.navx.getHeadingYaw();

			if (yaw > 0) power = power * -1;
		}
		else 							// Simple turn, full power until target angle reached.
		{
			yaw = RobotContainer.navx.getYaw();
			
			if (target < 0) power = power * -1;
		}
	}
	
	@Override
	public void execute() 
	{
		Util.consoleLog();
		
		if (usePid)
		{
			if (useHeading)
				yaw = RobotContainer.navx.getHeadingYaw();
			else
				yaw = RobotContainer.navx.getYaw();
			
			elapsedTime = Util.getElaspedTime();
			
			// Our target is zero yaw so we determine the difference between
			// current yaw and target and perform the PID calculation which
			// results in the speed of turn, reducing power as the difference
			// approaches zero. So our turn should slow and not overshoot. If
			// it does, the PID controller will reverse power and turn it back.
			// This continues until the error is within tolerance.
			
			power = pidController.calculate(yaw, elapsedTime);
			
			//power = pid.get();
			
			// When quick turn is true, first parameter is not used, power is fed to the
			// rate of turn parameter. PID controller takes care of the sign, that 
			// is the left/right direction of the turn.
			
			driveBase.curvatureDrive(0, power, true);
			
			Util.consoleLog("power=%.2f  hdg=%.2f  yaw=%.2f  err=%.2f  time=%f", power, 
							 RobotContainer.navx.getHeading(), yaw, pidController.getError(), elapsedTime); 
		}
		else if (useHeading)
		{
			driveBase.curvatureDrive(0, power, true);
			
			Util.consoleLog("yaw=%.2f  hdg=%.2f", yaw, RobotContainer.navx.getHeading());
			
			yaw = RobotContainer.navx.getHeadingYaw();
		}
		else 
		{
			driveBase.curvatureDrive(0, power, true);
			
			Util.consoleLog("yaw=%.2f  hdg=%.2f", yaw, RobotContainer.navx.getHeading());
			
			yaw = RobotContainer.navx.getYaw();
		}
	}
	
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();
		
		Util.consoleLog("after stop  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), yaw);

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", RobotContainer.navx.isRotating());
		
		//while (RobotContainer.navx.isRotating()) {Timer.delay(.10);}
		//Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("2  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), yaw);
	}
	
	@Override
	public boolean isFinished() 
	{
		if (usePid)
			return pidController.onTarget(kTolerance);			
		else if (useHeading)
			return Util.checkRange(yaw, 1.0);
		else 
			return Math.abs(yaw) >= Math.abs(target);
	}
	
	/** 
	 *Average left and right encoder counts to see how far robot has moved.
	 * @return Average encoder counts.
	 */
	public  int getEncoderCount()
	{
		return (driveBase.leftEncoder.get() + driveBase.rightEncoder.get()) / 2;
	}
	
	public enum Brakes
	{
		off,
		on
	}
	
	public enum Pid
	{
		off,
		on
	}
	
	public enum Heading
	{
		angle,
		heading
	}
	
	public enum StopMotors
	{
		dontStop,
		stop
	}	
}
