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

	private double			yaw, kSteeringGain = .10, elapsedTime = 0;
	private double			kP = .00005, kI = 0.00001, kD = 0.0;
	private double			power; 
	private int 			encoderCounts; 
	private StopMotors 		stop;
	private Brakes 			brakes;
	private Pid 			pid;
	private Heading 		heading;
	
	SynchronousPID			pidController = null;

	/**
	 * Creates a new AutoDrive command.
	 * 
	 * Auto drive straight in set direction and power for specified encoder count. Stops
	 * with or without brakes on CAN bus drive system. Uses NavX yaw to drive straight.
	 *
	 * @param driveBase The DriveBase subsystem used by this command to drive the robot.
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
	public AutoDrive(DriveBase driveBase, 
					 double power, 
					 int encoderCounts, 
					 StopMotors stop, 
					 Brakes brakes, 
					 Pid pid, 
					 Heading heading) 
	{
		this.driveBase = driveBase;

		Util.consoleLog("pwr=%.2f  count=%d  stop=%s  brakes=%s  pid=%s  hdg=%s", power, encoderCounts, stop, brakes, 
						pid, heading);

		Util.checkRange(power, 1.0);
		
		if (encoderCounts <= 0) throw new IllegalArgumentException("Encoder counts < 1");
			  
		this.power = power;
		this.encoderCounts = encoderCounts;
		this.stop = stop;
		this.brakes = brakes;
		this.pid = pid;
		this.heading = heading;
		
		kP = Math.abs(power) / encoderCounts;
		kI = kP / 10.0 * 2.0;
		
		Util.consoleLog("kP=%.5f  kI=%.5f", kP, kI);
		
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
				pidController.setSetpoint(-encoderCounts);
				pidController.setOutputRange(power, 0);
			}
			else
			{
				pidController.setSetpoint(encoderCounts);
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
		
		LCD.printLine(LCD_4, "Auto wheel encoder avg=%d", driveBase.getAvgEncoder());

		// Use PID to determine the power applied. Should reduce power as we get close
		// to the target encoder value.
		
		if (pid == Pid.on)
		{
			elapsedTime = Util.getElaspedTime();
			
			power = pidController.calculate(driveBase.getAvgEncoder(), elapsedTime);
			
			//power = pidController.get();
			
			Util.consoleLog("error=%.2f  power=%.2f  time=%f", pidController.getError(), power, elapsedTime);
		}
		else
			Util.consoleLog("tgt=%d  act=%d", encoderCounts, Math.abs(driveBase.getAvgEncoder()));

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
		
		int actualCount = Math.abs(driveBase.getAvgEncoder());
		
		Util.consoleLog("end: actual count=%d  error=%.2f pct", actualCount, 
				((double) actualCount - encoderCounts) / (double) encoderCounts * 100.0);
	}
	
	@Override
	public boolean isFinished() 
	{
		return Math.abs(driveBase.getAvgEncoder()) >= encoderCounts;	
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
