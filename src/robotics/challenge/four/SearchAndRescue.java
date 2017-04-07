package robotics.challenge.four;

import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class SearchAndRescue {

	public static void main(String[] args) {
	Behavior b1 = new Behavior() {
			
			@Override
			public boolean takeControl() {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void suppress() {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void action() {
				// TODO Auto-generated method stub
				
			}
		};
		
		Behavior b2 = new Behavior() {
			
			@Override
			public boolean takeControl() {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void suppress() {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void action() {
				// TODO Auto-generated method stub
				
			}
		};
		
		Behavior [] bArray = {b1,b2};
		
		Arbitrator arby = new Arbitrator(bArray);
		
		arby.start();

	}

}
