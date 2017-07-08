package robotics.challenge.four;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

/**
 * Behavior that follows a line on a bridge.
 * @author johan 
 *
 */
public class BridgeUp implements Behavior{
	boolean suppressed;
	
	EV3GyroSensor gyro;
	EV3ColorSensor color;
	
	final double THRESHOLD = 0.15;
	final int SPEED = 300;
	final double WHITE = 0.5;	
	final double BLACK = 0;	
	
	public BridgeUp(EV3ColorSensor color, EV3GyroSensor gyro)
	{
		suppressed = false;
		this.color = color;
		this.gyro = gyro;
	}
	
	@Override
	public boolean takeControl() 
	{
//		return true;
		float sampleGyro = readGyroRate();
		return sampleGyro <= -10 ; //FILL IN
	}
	
	@Override
	public void suppress() 
	{
		suppressed = true;
	}
	
	public void unsuppress()
	{
		suppressed = false;
	}
	
	public float readColorRedMode()
	{
		float[] sample = new float[1];
		SampleProvider sampleProvider = color.getRedMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	public float readGyroRate()
	{
		float[] sample = new float[1];
		SampleProvider sampleprovider = gyro.getRateMode();
		sampleprovider.fetchSample(sample, 0);
		return sample[0];
	} 
	
	public float readGyroAngle()
	{
		float[] sample = new float[1];
		SampleProvider sampleprovider = gyro.getAngleMode();
		sampleprovider.fetchSample(sample, 0);
		return sample[0];
	} 
	
	public double avgThreshold(double white, double black)
	{
		return ((white - black) / 2) + black;
	}
	
	public void motorsStop()
	{
		Motor.A.stop(true);
		Motor.C.stop(true);
	}
	
	public void motorsForward()
	{
		Motor.A.forward();
		Motor.C.forward();
	}
	
	public void motorsSpeed(int speedA, int speedC)
	{
		Motor.A.setSpeed(speedA);
		Motor.C.setSpeed(speedC);
	}
	
	public void turnLeft()
	{
		motorsSpeed(SPEED,SPEED);
		Motor.A.backward();
		Motor.C.forward();
		Delay.msDelay(500);
		motorsStop();
	}
	
	public void turnRight()
	{
		motorsSpeed(SPEED,SPEED);
		Motor.C.backward();
		Motor.A.forward();
		Delay.msDelay(500);
		motorsStop();
	}
	
	public void playSound()
	{
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		audio.systemSound(0);
	}
	
	@Override
	public void action() 
	{
		unsuppress();
		gyro.reset();
		playSound();
	}
}
