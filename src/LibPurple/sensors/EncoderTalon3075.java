package LibPurple.sensors;

import com.ctre.CANTalon;
<<<<<<< HEAD
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
=======
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713

import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderTalon3075 implements Encoder3075{
	
<<<<<<< HEAD
	private WPI_TalonSRX myTalon;
=======
	private CANTalon myTalon;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	private double dpp = 1;
	private double resetPoint = 0;
	private double inverted = 1;
	private static double encoderPulsesPerRotation = 4;
	PIDSourceType type = PIDSourceType.kDisplacement;
	
	private static double minimumValue = 0.0005;
	
	
<<<<<<< HEAD
	public EncoderTalon3075(WPI_TalonSRX talon)
=======
	public EncoderTalon3075(CANTalon talon)
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	{
		myTalon = talon;
	}
	
<<<<<<< HEAD
	public EncoderTalon3075(WPI_TalonSRX talon, boolean inverted)
=======
	public EncoderTalon3075(CANTalon talon, boolean inverted)
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	{
		myTalon = talon;
		this.inverted = inverted ? -1 : 1;
	}
	
	public void setPulsesPerRotation(double pulses)
	{
		encoderPulsesPerRotation = pulses;
	}
	
	public void setReverseDirection(boolean reverseDirection)
	{
		this.inverted = reverseDirection ? -1 : 1;
	}
	
	/**
	 * 
	 * @return Encoder's rate for 100ms.
	 */
	
	public double getRate()
	{
<<<<<<< HEAD
		double returnValue = (myTalon.getSelectedSensorVelocity(0) / dpp) * inverted * 10;
=======
		double returnValue = (myTalon.getSpeed() / dpp) * inverted * 10;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
		return Math.abs(returnValue) < minimumValue ? 0 : returnValue;
	}
	
	public double getRawSpeed()
	{
<<<<<<< HEAD
		return myTalon.getSelectedSensorVelocity(0) * inverted;
=======
		return myTalon.getSpeed() * inverted;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	}
	
	public double getRawPosition()
	{
<<<<<<< HEAD
		return myTalon.getSelectedSensorPosition(0) * inverted;
=======
		return myTalon.getPosition() * inverted;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	}
	
	public double getDistance()
	{
<<<<<<< HEAD
		double returnValue = (myTalon.getSelectedSensorPosition(0) - resetPoint) / dpp * inverted;
=======
		double returnValue = (myTalon.getPosition() - resetPoint) / dpp * inverted;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
		return Math.abs(returnValue) < minimumValue ? 0 : returnValue;
	}
	
	public void setDistancePerPulse(double dpp)
	{
		this.dpp = dpp;
	}
	
	public void reset()
	{
<<<<<<< HEAD
		resetPoint = myTalon.getSelectedSensorPosition(0);
=======
		resetPoint = myTalon.getPosition();
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return type;
	}

	@Override
	public double pidGet()
	{
		switch(type)
		{
		case kDisplacement: 
			return getDistance();
		case kRate:
			return getRate();
		default:
			break;
		}
		
		return getDistance();
	}

	@Override
	public void setPIDSourceType(PIDSourceType type) 
	{
		this.type = type;
	}
<<<<<<< HEAD
=======

	
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
}
