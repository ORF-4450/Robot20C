package Team4450.Robot20C.subsystems;

import static Team4450.Robot20C.Constants.*;

import Team4450.Lib.ValveDA;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class Climber
{
	private ValveDA			climberBrake = new ValveDA(CLIMBER_BRAKE_VALVE);
	private DigitalInput	winchSwitch = new DigitalInput(WINCH_SWITCH);

	// Encoder (regular type) is plugged into dio port n:
	// orange=+5v blue=signal, dio port n+1: black=gnd yellow=signal. 
	private Encoder			winchEncoder = new Encoder(WINCH_ENCODER, WINCH_ENCODER + 1, true, EncodingType.k4X);

	public Climber()
	{
		// TODO Auto-generated constructor stub
	}

}
