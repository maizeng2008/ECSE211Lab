/*
 * Navigation.java
 */
package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation implements UltrasonicController {
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 100;
	private static final double TILE_SIZE = 30.48;
	private static final int minDist = 15;        // if the the distance between robot and wall is smaller than 20cm, make a turn
	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;
	public static Odometer odo;
	public static double leftRadius;
	public static double rightRadius;
	public static double track;
	public static double current_T=0;
	public static double centerOfPlatformX = 1.0;//the center of the platform that we use for Lab3 X coordinate
	public static double centerOfPlatformY = 1.0;//the center of the platform that we use for Lab3 Y coordinate
	static boolean navigating = false;
	public static int distance = 130;//this distance is for US, 130 is from lab1's data which is the max distance that the sensor cannot sense 
	//private static int smallMovementCount = 0;

	SensorModes usSensor;
	private float[] usData;
	private SampleProvider us_sensor;
	private EV3MediumRegulatedMotor sensorMotor;


	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double track, SensorModes usSensor,float[] usData, SampleProvider us_sensor) throws OdometerExceptions {
		Navigation.leftMotor=leftMotor;
		Navigation.rightMotor=rightMotor;
		Navigation.leftRadius= leftRadius;
		Navigation.rightRadius= rightRadius;
		this.usSensor=usSensor;
		Navigation.track= track;
		this.usData=usData;
		this.us_sensor=us_sensor;
		this.odo = Odometer.getOdometer();              //give access to present odometer.
	}

	/**
	 * This method is meant to drive the robot to the predefined waypoints.
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {
		double minimalT=0,travelDistance=0;
		double currentX=0.0;
		double currentY=0.0;
		double odometer[] = { 0, 0, 0 };

		navigating = true;
		// Get odometer readings
		try {
			odometer = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e) {
			// Do nothing lol
			e.printStackTrace();
		}

		// Set odometer reading angle as prev angle as well
		current_T = odometer[2];

		// Get displacement to travel on X and Y axis
		currentX= odometer[0];
		currentY = odometer[1];


		//Getting the distances with respect to the tile size
		double deltaX = x*TILE_SIZE-currentX;
		double deltaY =	y*TILE_SIZE-currentY;
		travelDistance = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));

		//Calculating the minimal angle to get to destination
		minimalT = Math.toDegrees(Math.atan2(deltaX,deltaY));


		//If the angle is negative, we want its positive equivalent
		if(minimalT < 0) {
			minimalT = 360 - Math.abs(minimalT);
		}

		//Calling the rotate
		turnTo(minimalT,current_T);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(leftRadius, travelDistance), true);
		rightMotor.rotate(convertDistance(rightRadius, travelDistance), true);

		while(leftMotor.isMoving() && rightMotor.isMoving())
		{
			if(iAmBlocked())
				obstacleAvoid(x, y, whichWay());
			else
				continue;	
		}

		//tell that this method has stopped
		navigating = false;

	}


	/**
	 * This method is meant to drive the robot in a square of size 2x2 Tiles. It is to run in parallel
	 * with the odometer and Odometer correcton classes allow testing their functionality.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param leftRadius
	 * @param rightRadius
	 * @param width
	 */
	public void turnTo(double minimalT, double original) {
		//Calculating by how much we have to rotate with respect to our current angle
		double deltaT = 0;
		deltaT= minimalT - original;
		boolean turnLeft = false;

		//Getting the positive equivalent
		if(deltaT < 0) {
			deltaT = 360 - Math.abs(deltaT); 
		}

		if(deltaT > 180) {
			//Turn left
			turnLeft = true;
			//Rotate by
			deltaT = 360 - Math.abs(deltaT); 
		}
		else {
			turnLeft= false;
		}

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if(turnLeft) {
			leftMotor.rotate(-convertAngle(leftRadius, track, deltaT), true);
			rightMotor.rotate(convertAngle(rightRadius, track, deltaT), false);
		}
		else {
			leftMotor.rotate(convertAngle(leftRadius, track, deltaT), true);
			rightMotor.rotate(-convertAngle(rightRadius, track, deltaT), false);
		}

	}

	/**
	 * tell the robot is it navigating
	 * @return
	 */
	public static boolean isNavigating() {
		return navigating;
	}

	/**
	 * This function is to get the distance from the UltrasonicController class
	 * @param distance
	 */
	public void processUSData(int distance)
	{
		Navigation.distance = distance;
	}


	/**
	 * this function is to get the distance between US and wall
	 * this is for convenient
	 * @return
	 */
	public int getUSdata()
	{
		return Navigation.distance;
	}

	/**
	 * This method is to accomplish the goal of avoid obstacle
	 * method is simple
	 * it will first move forward to a certain distance and then look back to see whether there is a wall
	 * if there is a wall then turn to move forward again
	 * if not move forward directly
	 * it will only make a 90 degree turn, the matter is to turn right or left
	 * our boolean turnRight is going to tell the robot whether it should turn left or right
	 * this boolean is computed in the method whichWay()
	 * @param theX
	 * @param theY
	 * @param turnRight
	 */
	public void obstacleAvoid(double theX, double theY, boolean turnRight)
	{
		//in this method, it only will make the robot to turn 90 degree or -90 degree ( 90 degree eventually)
		//but the only matter is to turn right or left
		//if it want to turn right then turnAngle will be +90 degree
		//if it want to turn left then turnAngle will be -90.0 degree
		int turnAngle = 0;
		if(turnRight) turnAngle = 90;
		else turnAngle = -90;
		int original = (int) odo.getXYT()[2];
		if(original > 180)
			original = 360 - original;
		turnTo(original + turnAngle, original);
		turnAndMoveForWard(30, turnAngle);
		turnTo(original - turnAngle, original);
		turnAndMoveForWard(58, turnAngle);
		turnTo(original - turnAngle, original);
		turnAndMoveForWard(30, turnAngle);
		turnTo(original + turnAngle, original);
	}


	/**
	 * this method is for the robot to avoid the obstacle
	 * use the same idea of letting the robot move forward
	 * @param shortLenOfBrick
	 * @param turnAngle
	 * @return
	 */
	private void turnAndMoveForWard(int shortLenOfBrick, double turnAngle) {
		// TODO Auto-generated method stub

		//turn the angle to the correct direction
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// Do nothing lol
			e.printStackTrace();
		}

		//turnTo(turnAngle + odo.getXYT()[2]);

		//move forward
		double radOfWheel = 2.2; // this is the radius of the wheel
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(radOfWheel, shortLenOfBrick), true);
		rightMotor.rotate(convertDistance(radOfWheel, shortLenOfBrick), false);

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	/**
	 * this method is to determine whether the robot is blocked or not
	 * @return
	 */
	public boolean iAmBlocked()
	{

		//us_sensor.fetchSample(usData,0);
		//distance=(int) (usData[0]*100);
		if(distance < minDist) 
		{
			return true;
		}
		else 
			return false;
	}


	/**
	 * this method is to determine whether the robot should turn right or left after it is blocked
	 * if return true then turn right if return false then turn left
	 * the idea is to use the same algorithm of travelTo function
	 * but now our goal is to find the shortest way from our current location to the center of platform which is (1,1)
	 * @return
	 */
	private boolean whichWay() {
		// TODO Auto-generated method stub
		double xRightNow = odo.getXYT()[0];
		double yRightNow = odo.getXYT()[1];
		double thetaRightNow = odo.getXYT()[2];
		double dt;
		double thetaToJudge = 0;

		double relativeY =	centerOfPlatformY*TILE_SIZE-yRightNow;
		double relativeX = centerOfPlatformX*TILE_SIZE - xRightNow;
		thetaToJudge = Math.toDegrees(Math.atan2(relativeX,relativeY));
		if(thetaToJudge < 0)
			thetaToJudge = 360 - Math.abs(thetaToJudge);

		dt = thetaToJudge - thetaRightNow;

		if(dt < 0)
			dt = 360 + dt;


		if(dt > 180)
			return false;//left           

		else
			return true;//right

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

	@Override
	public int readUSDistance() {
		return Navigation.distance;
	}


}
