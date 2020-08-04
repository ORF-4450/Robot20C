package Team4450.Robot20C.commands;

import Team4450.Lib.LCD;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import static Team4450.Robot20C.Constants.*;
import Team4450.Robot20C.RobotContainer;
import Team4450.Robot20C.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDrive extends CommandBase
{
	private final DriveBase driveBase;

	double			yaw, kSteeringGain = .10, elapsedTime = 0;
	double			kP = .00015, kI = 0.000015, kD = 0.0;
	double			power; 
	int 			encoderCount; 
	StopMotors 		stop;
	Brakes 			brakes;
	Pid 			pid;
	Heading 		heading;
	
	SynchronousPID	pidController = null;

	/**
	 * Creates a new AutoDrive command.
	 * 
	 * Auto drive straight in set direction and power for specified encoder count. Stops
	 * with or without brakes on CAN bus drive system. Uses NavX yaw to drive straight.
	 *
	 * @param subsystem The subsystem used by this command.
	 * @param power Power applied, + is forward.
	 * @param encoderCounts Target encoder counts to move, always +.
	 * @param stop Stop stops motors at end of move, dontStop leaves power on to flow into next move.
	 * @param brakes Brakes on or off.
	 * @param pid On is use PID to control movement, off is simple drive.
	 * @param heading Heading is measure steering yaw from last set navx target heading, angle is measure yaw
	 * from direction robot is pointing when driving starts.
	 * 
	 * Note: This routine is designed for tank drive and the P,I,D,steering gain values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to stop smoothly
	 * and accurately.
	 */
	public AutoDrive(DriveBase subsystem, 
					 double power, 
					 int encoderCounts, 
					 StopMotors stop, 
					 Brakes brakes, 
					 Pid pid, 
					 Heading heading) 
	{
		Util.consoleLog();
		
		driveBase = subsystem;

		Util.consoleLog("pwr=%.2f  count=%d  stop=%s  brakes=%s  pid=%s  hdg=%s", power, encoderCounts, stop, brakes, 
						pid, heading);

		Util.checkRange(power, 1.0);
		
		if (encoderCounts <= 0) throw new IllegalArgumentException("Encoder counts < 1");
			  
		this.power = power;
		this.encoderCount = encoderCounts;
		this.stop = stop;
		this.brakes = brakes;
		this.pid = pid;
		this.heading = heading;
		
		addRequirements(this.driveBase);
	}
	
	@Override
	public void initialize()
	{
		Util.consoleLog();
		
		if (brakes == Brakes.on)
			driveBase.SetCANTalonBrakeMode(true);
		else
			driveBase.SetCANTalonBrakeMode(false);
			
		driveBase.resetEncodersWithDelay();
		
		// If not measuring yaw from current heading, reset yaw based on current direction robot is facing.
		
		if (heading == Heading.angle)
		{
			Util.consoleLog("before reset=%.2f  hdg=%.2f", RobotContainer.navx.getYaw(), RobotContainer.navx.getHeading());
			
			RobotContainer.navx.resetYawWait(1, 500);
			
			Util.consoleLog("after reset2=%.2f  hdg=%.2f", RobotContainer.navx.getYaw(), RobotContainer.navx.getHeading());
		}
		
		// If using PID to control distance, configure the PID object.
		
		if (pid == Pid.on)
		{
			pidController = new SynchronousPID(kP, kI, kD);
			
			if (power < 0)
			{
				pidController.setSetpoint(-encoderCount);
				pidController.setOutputRange(power, 0);
			}
			else
			{
				pidController.setSetpoint(encoderCount);
				pidController.setOutputRange(0, power);
			}

			// Start elapsed time tracking.
			
			Util.getElaspedTime();
		}
	}
	
	@Override
	public void execute() 
	{
		Util.consoleLog();
		
		LCD.printLine(LCD_4, "Auto wheel encoder avg=%d", getEncoderCount());

		// Use PID to determine the power applied. Should reduce power as we get close
		// to the target encoder value.
		
		if (pid == Pid.on)
		{
			elapsedTime = Util.getElaspedTime();
			
			pidController.calculate(getEncoderCount(), elapsedTime);
			
			power = pidController.get();
			
			Util.consoleLog("error=%.2f  power=%.2f  time=%f", pidController.getError(), power, elapsedTime);
		}
		else
			Util.consoleLog("tgt=%d  act=%d", encoderCount, Math.abs(getEncoderCount()));

		// Yaw angle is negative if robot veering left, positive if veering right when going forward.
		
		if (heading == Heading.heading)
			yaw = RobotContainer.navx.getHeadingYaw();
		else
			yaw = RobotContainer.navx.getYaw();
		
		LCD.printLine(LCD_5, "yaw=%.2f", yaw);
		
		Util.consoleLog("yaw=%.2f  hdg=%.2f  rot=%.2f", yaw, RobotContainer.navx.getHeading(), -yaw * kSteeringGain);
		
		// Note we invert sign on the angle because we want the robot to turn in the opposite
		// direction than it is currently going to correct it. So a + angle says robot is veering
		// right so we set the turn value to - because - is a turn left which corrects our right
		// drift. kSteeringGain controls how aggressively we turn to stay on course.
		
		driveBase.curvatureDrive(power, Util.clampValue(-yaw * kSteeringGain, 1.0), false);
	}
	
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		if (stop == StopMotors.stop) driveBase.stop();
		
		double actualCount = Math.abs(getEncoderCount());
		
		Util.consoleLog("end: actual count=%d  error=%.3f%", actualCount, 
				(actualCount - encoderCount) / encoderCount * 100);
	}
	
	@Override
	public boolean isFinished() 
	{
		return Math.abs(getEncoderCount()) >= encoderCount;	
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
