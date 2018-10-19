package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {
		// Motor Objects, and Robot related parameters
		private static final EV3LargeRegulatedMotor leftMotor =
				new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		private static final EV3LargeRegulatedMotor rightMotor =
				new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
		private static final TextLCD lcd = LocalEV3.get().getTextLCD();
		private static final EV3ColorSensor LS = 
				new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	
		static SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4")); // usSensor is the instance
		static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		static float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		// returned
	
		//initialize all the value that I need for Lab4
		private static final int FORWARD_SPEED = 150;
		private static final int ROTATE_SPEED = 100;//set rotate speed smaller than previous in order to let the US do a better job
		private static final double TILE_SIZE = 30.48;
		public static final double WHEEL_RAD = 2.2;
		public static final double TRACK = 11.3;
		
		  public static void main(String[] args) throws OdometerExceptions {
			  
			    int buttonChoice;
			    int chooseWhichRoutine;//if chooseWichEdge is equal to 0, then it is rising edge, else it is falling edge
			    
			    // Odometer related objects
			    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation

			    Display odometryDisplay = new Display(lcd); // No need to change
			  
			    // Setup Ultrasonic Poller  //copy and paste from previous lab
			    UltrasonicPoller usPoller = null; 
			    
			    do {
			        // clear the display
			        lcd.clear();
			        
			        // ask the user whether the motors should drive in a square or float
			        lcd.drawString("< Left | Right >", 0, 0);
			        lcd.drawString("       |        ", 0, 1);
			        lcd.drawString("  Rise | Fall  ", 0, 2);

			        
			        buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			    }while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
			    
			    //see which mode are we in, choose rise is to 
			    if(buttonChoice == Button.ID_LEFT)
			    		chooseWhichRoutine = 0;//if chooseWhichEdge is equal to 0, then it is rising edge
			    else
			    		chooseWhichRoutine = 1;//if chooseWhichEdge is equal to 1, then it is falling edge
			    
			    //copy and paste from Lab2 Lab2 class, start the odometer
			    // Display changes in position as wheels are (manually) moved
			    Thread odoThread = new Thread(odometer);
			    odoThread.start();
			    Thread odoDisplayThread = new Thread(odometryDisplay);
			    odoDisplayThread.start();
			    final Navigation Navigator = new Navigation(leftMotor,rightMotor,WHEEL_RAD,WHEEL_RAD,TRACK,usSensor,usData,usDistance);
			    
			    final UltrasonicLocalizer USLocalizer = new UltrasonicLocalizer(Navigator,chooseWhichRoutine);
			    final LightLocalizer LSLocalizer = new LightLocalizer(Navigator,LS);
			    usPoller = new UltrasonicPoller(usDistance, usData,Navigator);; // the selected controller on each cycle
			    usPoller.start();
			    
			    odometer.setXYT(0, 0, 0);
			    
			    // Sleep for 2 seconds
			    try {
			      Thread.sleep(2000);
			    } catch (InterruptedException e) {
			      // There is nothing to be done here
			    }
			    
			    (new Thread() {
			        public void run() {
			        
			          USLocalizer.whichRoutine();
			          while (Button.waitForAnyPress() != Button.ID_ENTER) {
			        	  	
			          }
			          LSLocalizer.lightLocalize();
			        } 
			      }).start();
			    
			    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			    System.exit(0);
			   
		  }
}
