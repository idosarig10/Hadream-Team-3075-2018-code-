package LibPurple.sensors;

<<<<<<< HEAD
import com.ctre.CANTalon;
<<<<<<< HEAD
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
=======
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
>>>>>>> latesttryupdate

import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderTalon3075 implements Encoder3075{
	
<<<<<<< HEAD
<<<<<<< HEAD
	private WPI_TalonSRX myTalon;
=======
	private CANTalon myTalon;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
	private double dpp = 1;
	private double resetPoint = 0;
	private double inverted = 1;
	private static double encoderPulsesPerRotation = 4;
=======
	private WPI_TalonSRX myTalon;
	private double dpp = 1;
	private double resetPoint = 0;
	private double inverted = 1;
>>>>>>> latesttryupdate
	PIDSourceType type = PIDSourceType.kDisplacement;
	
	private static double minimumValue = 0.0005;
	
	
<<<<<<< HEAD
<<<<<<< HEAD
	public EncoderTalon3075(WPI_TalonSRX talon)
=======
	public EncoderTalon3075(CANTalon talon)
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
	public EncoderTalon3075(WPI_TalonSRX talon)
>>>>>>> latesttryupdate
	{
		myTalon = talon;
	}
	
<<<<<<< HEAD
<<<<<<< HEAD
	public EncoderTalon3075(WPI_TalonSRX talon, boolean inverted)
=======
	public EncoderTalon3075(CANTalon talon, boolean inverted)
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
	public EncoderTalon3075(WPI_TalonSRX talon, boolean inverted)
>>>>>>> latesttryupdate
	{
		myTalon = talon;
		this.inverted = inverted ? -1 : 1;
	}
	
<<<<<<< HEAD
	public void setPulsesPerRotation(double pulses)
	{
		encoderPulsesPerRotation = pulses;
	}
	
=======
>>>>>>> latesttryupdate
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
<<<<<<< HEAD
		double returnValue = (myTalon.getSelectedSensorVelocity(0) / dpp) * inverted * 10;
=======
		double returnValue = (myTalon.getSpeed() / dpp) * inverted * 10;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
		double returnValue = (myTalon.getSelectedSensorVelocity(0) / dpp) * inverted * 10;
>>>>>>> latesttryupdate
		return Math.abs(returnValue) < minimumValue ? 0 : returnValue;
	}
	
	public double getRawSpeed()
	{
<<<<<<< HEAD
<<<<<<< HEAD
		return myTalon.getSelectedSensorVelocity(0) * inverted;
=======
		return myTalon.getSpeed() * inverted;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
		return myTalon.getSelectedSensorVelocity(0) * inverted;
>>>>>>> latesttryupdate
	}
	
	public double getRawPosition()
	{
<<<<<<< HEAD
<<<<<<< HEAD
		return myTalon.getSelectedSensorPosition(0) * inverted;
=======
		return myTalon.getPosition() * inverted;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
		return (myTalon.getSelectedSensorPosition(0)-resetPoint) * inverted;
>>>>>>> latesttryupdate
	}
	
	public double getDistance()
	{
<<<<<<< HEAD
<<<<<<< HEAD
		double returnValue = (myTalon.getSelectedSensorPosition(0) - resetPoint) / dpp * inverted;
=======
		double returnValue = (myTalon.getPosition() - resetPoint) / dpp * inverted;
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
		double returnValue = (myTalon.getSelectedSensorPosition(0) - resetPoint) / dpp * inverted;
>>>>>>> latesttryupdate
		return Math.abs(returnValue) < minimumValue ? 0 : returnValue;
	}
	
	public void setDistancePerPulse(double dpp)
	{
		this.dpp = dpp;
	}
	
	public void reset()
	{
<<<<<<< HEAD
<<<<<<< HEAD
		resetPoint = myTalon.getSelectedSensorPosition(0);
=======
		resetPoint = myTalon.getPosition();
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
		resetPoint = myTalon.getSelectedSensorPosition(0);
>>>>>>> latesttryupdate
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
<<<<<<< HEAD
=======

	
>>>>>>> 29cf4a5bc79be43519cab52197919762ce787713
=======
>>>>>>> latesttryupdate
}
