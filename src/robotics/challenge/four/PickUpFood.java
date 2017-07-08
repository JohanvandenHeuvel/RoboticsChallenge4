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
import lejos.utility.Delay;

/**
 * Behavior that picks up food.
 * @author johan
 *
 */
public class PickUpFood implements Behavior{
	boolean suppressed;
	boolean inRange = false;
	boolean pickedup = false;
	
	EV3GyroSensor gyro;
	EV3UltrasonicSensor sonic;
	EV3ColorSensor color;
	
	final double THRESHOLD = 0.04;
	final int SPEED = 150;
	final int RED = 0;
	final int BLUE = 2;
	
	final double WHITE = 0.3;		
	final double BLACK = 0.05;	
	
	public PickUpFood(EV3GyroSensor gyro, EV3ColorSensor color, EV3UltrasonicSensor sonic) 
	{
		suppressed = false;
		this.sonic = sonic;
		this.color = color;
		this.gyro = gyro;
	}
	
	@Override
	public boolean takeControl() 
	{
		float sample = readUltraSonic();
		return sample < THRESHOLD && !pickedup;
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
		SampleProvider sampleprovider = gyro.getAngleMode();
		sampleprovider.fetchSample(sample, 0);
		return sample[0];
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
//		File file = new File("sound.wav");
//		System.out.println(file.exists());
//		System.out.println(Sound.playSample(file, 100));
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
	
	public double avgThreshold(double white, double black)
	{
		return ((white - black) / 2) + black;
	}
	
	public float readColorRedMode()
	{
		float[] sample = new float[1];
		SampleProvider sampleProvider = color.getRedMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	@Override
	public void action() {
		unsuppress();
		motorsStop();
		
		Motor.B.setSpeed(100);
		Motor.B.forward();
		Delay.msDelay(10000);
		
		playSound();
		Motor.B.stop();
		
		//Color values
		double avgThreshold = avgThreshold(WHITE, BLACK);
		double lastSample = readColorRedMode();
		
		//PID-controller values
		double Kp = 1000; 		//change
		
		while (true) 
		{
			float angle = readGyroAngle();
//			System.out.println(angle + " " + Challenge4.bridgeCrossed);
//			if(angle <= -8)
//			{
////				playSound();
//				bridgeDown = true;
//			}
//			if(bridgeDown && angle >= -2)
//			{
//				playSound();
//				Challenge4.bridgeCrossed = true;
//				bridgeDown = false;
//				gyro.reset();
//			}
			
			float newSample = readColorRedMode();
			float avgSample = (float) ((newSample + lastSample) / 2);
			lastSample = newSample;
			
			//PID-controller calculations
			double newError = avgThreshold - avgSample;
			
			//Normal PID-controller behavior
			int correction = (int) (Kp * newError);

			
//			//Turn faster if outside Bounds
			double lowerBound = 0.10; //0.35 * avgThreshold;
			double upperBound = 0.25; //1.35 * avgThreshold;
			
//			motorsSpeed(SPEED + correction, SPEED - correction);
//			motorsForward();
			
			if (avgSample < lowerBound)
			{
				//Turn right if on black
				motorsSpeed(SPEED + correction, SPEED - correction);
				Motor.C.backward();
				Motor.A.forward();
			}
//			else if (avgSample >= upperBound)
//			{
//				//Turn left if on middle of tape
//				motorsSpeed(SPEED - correction, SPEED + correction);
//				Motor.A.backward();
//				Motor.C.backward();
//			}
			else
			{
				motorsSpeed(SPEED + correction, SPEED - correction);
				motorsForward();
			}
		}
	}
}
