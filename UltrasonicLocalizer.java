package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;

public class UltrasonicLocalizer {
		private static final int ROTATE_SPEED = 100;
		private static final	 double wheelRadius = 2.2;
		private static final double track = 11.3;
		private Navigation Navigator = null;
		/*
		 * have to mention that k and d are gotten from a lot of experiments but it is not hard code
		 * because as the slides said, they are some values from experiments
		 */
		int k = 1; //noise margin 
		int d = 40; //as localize tutorial slides said
		private Odometer odo; //get the current local position data
		private int chooseWhichRoutine = -1;//if chooseWichEdge is equal to 0, then it is rising edge, if 1 it is falling edge, intial it as -1 so that it will confused by 0 or 1
	
		public UltrasonicLocalizer(Navigation Navigator, int chooseWhichRoutine) throws OdometerExceptions {
			this.Navigator=Navigator;
			this.chooseWhichRoutine=chooseWhichRoutine;  
			this.odo = Odometer.getOdometer();
		}

	  /**
	   * this method is to process the input that the user just gave,
	   * whether using a routine of falling edge or a routine of rising edge
	   * if the chooseWhichRoutine == 0 then use the rising edge routine
	   * else use a falling edge routine
	   * this is chosen on the EV3 display
	   */
	  public void whichRoutine()
	  {
		  if(chooseWhichRoutine == 0)
			  risingEdgeRoutine();
		  else
			  fallingEdgeRoutine();
	  }
	  
	  /**
	   * this method is implemented as the slides said 
	   * If the robot starts facing a wall, it can:
	   * Detect a rising edge, switch directions, then detect another rising edge
	   * Detect a rising edge, continue in the same direction, then detect a falling edge
	   */
	  private void risingEdgeRoutine() {
		  double a = 0;
		  double b = 0;
		  rotateTheRobot(true, 360 , true);
		  
		  while(true)
		  {
			  if(Navigator.readUSDistance() > d+k)
				  break;
		  }
		  
		  Sound.beep();

		  stopMotor();

		  a = odo.getXYT()[2];//get a
		 	  
		  rotateTheRobot(false,45,false);
		  
		  //now get b
		  rotateTheRobot(false,360,true);//let it roll to detect a falling edge again

		  while(Navigator.readUSDistance() < d+k ) {

		  } 
		    
		  stopMotor();
		  Sound.beep();

		  b = 360 - odo.getXYT()[2];
		  
		  double deltaTheta = (a+b)/2;
		  
		  stopMotor();
		  rotateTheRobot(true,deltaTheta,false);
		  stopMotor();
		  rotateTheRobot(true,135,false);
		  odo.setTheta(0);
	  }

	  /**
	   * this method is implemented as the slides said 
	   * If the robot starts facing away from the walls, it can:
	   * Detect a falling edge, continue in the same direction, then detect a rising edge
	   */
	private void fallingEdgeRoutine() {
		  // TODO Auto-generated method stub
			double a = 0;
			double b = 0;

			rotateTheRobot(true,360,true);//let it roll for 2 circle, this will make sure that it will finish the data collecting
			
			while(true) {
				if(Navigator.readUSDistance() < d-k)   //detect a rising edge  
					break; 
			}
			Sound.beep();//sound buzz to let user know it have detect a rising edge
		    stopMotor();
			a = odo.getXYT()[2];//get a
			rotateTheRobot(false,45,false);
			//now get b
			rotateTheRobot(false,360,true);//let it roll to detect a falling edge again
			
			while(Navigator.readUSDistance() > d-k) {//continue to travel until it detect a falling edge
	
			}
			stopMotor();
			Sound.beep();
			b = 360 - odo.getXYT()[2];//what we want acutally is 360 - odo.getXY()[2] but for convenience i use this
			
			double deltaTheta = (a+b)/2;//now it is some value but it will process on robot to let it point to the 45 degree on the board
			
		    stopMotor();
			rotateTheRobot(true,deltaTheta,false);
			stopMotor();
			rotateTheRobot(false,45,false);
			odo.setTheta(0);
	  }

	  /**
	   * this method is created to let the robot rotate certain angle
	   * @param cwOrCcw
	   * @param angleToRotate
	   */
	  private void rotateTheRobot(boolean cwOrCcw, double angleToRotate, boolean blocked) {
		  double absAngleToRotate = Math.abs(angleToRotate);
		  Navigation.leftMotor.setSpeed(ROTATE_SPEED);
		  Navigation.rightMotor.setSpeed(ROTATE_SPEED);
		  
		  //just a reminder that if want the robot to rotate clockwise then leftMotor rotate is positive and right is negative
		  //true means cw
		  //false means ccw
		  if(cwOrCcw) {
			  Navigation.leftMotor.rotate(convertAngle(wheelRadius, track, absAngleToRotate),true);
			  Navigation.rightMotor.rotate(-convertAngle(wheelRadius, track, absAngleToRotate),blocked);
		  }
		  else
		  {
			  Navigation.leftMotor.rotate(-convertAngle(wheelRadius, track, absAngleToRotate),true);
			  Navigation.rightMotor.rotate(convertAngle(wheelRadius, track, absAngleToRotate),blocked);
		  }
		  // TODO Auto-generated method stub
	  }

	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	  private static int convertDistance(double radius, double travelDistance) {
	    return (int) ((180.0 * travelDistance) / (Math.PI * radius));
	  }

	  private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }  
	  
	  /**
	   * this method is to stip the motor ,both of them, left and right
	   */
	  private void stopMotor() {
		// TODO Auto-generated method stub
		  Navigator.leftMotor.setSpeed(0);
		  Navigator.rightMotor.setSpeed(0);	
	}
}
