package robotics.challenge.four;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Stopwatch;

/**
 * Behavior that finds a pillar.
 * @author johan
 *
 */
public class FindPillar implements Behavior{
	boolean suppressed;
	boolean inRange = false;
	
	String previousturn = "right";
	int time = 100;
	
//	EV3GyroSensor gyro;
//	EV3UltrasonicSensor sonic;
//	EV3ColorSensor color;
//	
	SampleProvider gyro;
	SampleProvider sonic;
	SampleProvider colorID;
	
	final double THRESHOLD = 0.01;
	final int SPEED = 25;
	final int RED = 5;
	final int BLUE = 2;
	
	
	
	public FindPillar(EV3GyroSensor gyro, EV3ColorSensor color, EV3UltrasonicSensor sonic) 
	{
		suppressed = false;
//		this.sonic = sonic;
//		this.color = color;
//		this.gyro = gyro;
		
		this.gyro = gyro.getAngleMode();
		this.sonic = sonic.getDistanceMode();
		this.colorID = color.getColorIDMode();
	}
	
	@Override
	public boolean takeControl() 
	{
		return true;
//		float sampleGyro = readGyroAngle();
//		return Math.abs(sampleGyro) >= 450 ;
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
	
	public float readGyroAngle()
	{
		float[] sample = new float[1];
//		SampleProvider sampleprovider = gyro.getAngleMode();
		gyro.fetchSample(sample, 0);
		return sample[0];
	} 
	
	public float readUltraSonic()
	{
		float[] sample = new float[1];
//		SampleProvider sampleProvider = sonic.getDistanceMode();
		sonic.fetchSample(sample, 0);
		return sample[0];
	}
	
	public float readColorIDMode()
	{
		float[] sample = new float[1];
//		SampleProvider sampleProvider = color.getColorIDMode();
		colorID.fetchSample(sample, 0);
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
	
	public void motorsSpeed(int speedA, int speedC)
	{
		Motor.A.setSpeed(speedA);
		Motor.C.setSpeed(speedC);
	}

	@Override
	public void action() {
		unsuppress();
		
		System.out.println("FindPillar");
		
		playSound();
		
		float oldSample = readUltraSonic();
		
		while (!suppressed) {
			float newSample = readUltraSonic();
			float sampleUltraSonic = (oldSample + newSample) / 2;
			
			System.out.println(sampleUltraSonic);
			
			if(sampleUltraSonic < THRESHOLD)
			{
				inRange();
			}
			else
			{
				/**
				 * Turn if no object in range
				 * Forward if object in range
				 */
				if (sampleUltraSonic > 0.5)
				{
					if (previousturn.equals("left"))
					{
						Stopwatch sw = new Stopwatch();
						while(sw.elapsed() < time)
						{
							if(readUltraSonic() < 0.5)
							{
								playSound();
								time = 100;
								break;
							}
								
							motorsSpeed(SPEED,SPEED);
							Motor.C.backward();
							Motor.A.forward();
						}
						sw.reset();
						motorsStop();
						time = time + time;
						previousturn = "right";
					}
					if (previousturn.equals("right"))
					{
						Stopwatch sw = new Stopwatch();
						while(sw.elapsed() < time)
						{
							if(readUltraSonic() < 0.5)
							{
								playSound();
								time = 100;
								break;
							}
							motorsSpeed(SPEED,SPEED);
							Motor.A.backward();
							Motor.C.forward();
						}
						sw.reset();
						motorsStop();
						time = time + time;
						previousturn = "left";
					}
//					motorsSpeed(SPEED, (int) 0.5 * SPEED);
//					Motor.A.backward();
//					Motor.C.backward();
				}
//				else if (oldSample < newSample)
//				{
//					motorsSpeed((int) (2*SPEED), (int) (1.5*SPEED));
//					motorsForward();
//				}
				else 
				{
					time = 100;
					motorsSpeed((int) (2*SPEED), 2*SPEED);
					motorsForward();
				}
			}
			oldSample = newSample;
		}
		motorsStop();
	}
}
