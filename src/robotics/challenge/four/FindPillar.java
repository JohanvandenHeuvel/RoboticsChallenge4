package robotics.challenge.four;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

/**
 * Behavior that finds a pillar.
 * @author johan
 *
 */
public class FindPillar implements Behavior{
	boolean suppressed;
	boolean inRange = false;
	
	EV3UltrasonicSensor sonic;
	EV3ColorSensor color;
	
	final double THRESHOLD = 0.08;
	final int SPEED = 200;
	final int RED = 5;
	final int BLUE = 2;
	
	public FindPillar(EV3ColorSensor color, EV3UltrasonicSensor sonic) 
	{
		suppressed = false;
		this.sonic = sonic;
		this.color = color;
	}
	
	@Override
	public boolean takeControl() 
	{
		return !inRange;
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
	
	public float readUltraSonic()
	{
		float[] sample = new float[1];
		SampleProvider sampleProvider = sonic.getDistanceMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	public float readColorIDMode()
	{
		float[] sample = new float[1];
		SampleProvider sampleProvider = color.getColorIDMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	public void playSound()
	{
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		audio.systemSound(0);
	}
	
	public void inRange()
	{
		inRange = true;
		suppress();
		playSound();
		
		float sampleColor = readColorIDMode();
		if (sampleColor == RED)
			System.out.println("RED");
		if (sampleColor == BLUE)
			System.out.println("BLUE");
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
	
	@Override
	public void action() {
		unsuppress();
		
		while (!suppressed) {
			float sampleUltaSonic = readUltraSonic();
			
			if(sampleUltaSonic < THRESHOLD)
			{
				inRange();
			}
			else
			{
				int speedMotorA = (sampleUltaSonic > 100) ? 0 : SPEED;
				int speedMotorC = SPEED;
						
				Motor.A.setSpeed(speedMotorA);
				Motor.C.setSpeed(speedMotorC);

				motorsForward();
			
				Thread.yield();
			}
		}
		
		motorsStop();
	}
}
