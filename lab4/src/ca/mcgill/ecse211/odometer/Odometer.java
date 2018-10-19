/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms
  
  private double oldTachoR;
  private double oldTachoL;
  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
    
  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   * Implemented by group 37
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();
      //Calculates how many degrees each wheel has rotated by. 
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      //Calculate new robot position based on tachometer counts
      //initialize changes and go get present theta value
      double dx=0;
      double dy=0;
      double dt=0;
      double currentTheta = odo.getXYT()[2];//get the theta
      
      //based on slides lec04 16/21 and 20/21
      double changeTachoL = leftMotorTachoCount - oldTachoL;
      double changeTachoR = rightMotorTachoCount - oldTachoR;
       
      //TachoChangeForForward means that the half of the common of the change of left tacho and the change of the right tacho is the tacho used to go straight
      //TachoChangeForTurning means that the half of the diff of the change of the left tacho and the change of the right tacho is the tcho iused to turn
      //can be proved using basic math.
      //
      double TachoChangeForForward = (changeTachoL + changeTachoR)/2;
      double TachoChangeForTurnning = (changeTachoL - changeTachoR)/2;
 
      
      //refresh our current theta, our angle
      currentTheta = currentTheta + dt;
      //from slides 15/21
      dy = (WHEEL_RAD * TachoChangeForForward)*(Math.PI/180)*Math.cos(Math.PI*currentTheta/180);
      dx = (WHEEL_RAD * TachoChangeForForward)*(Math.PI/180)*Math.sin(Math.PI*currentTheta/180);
      
      //Calculating instantaneous displacement from measurements
      dt = (WHEEL_RAD * TachoChangeForTurnning)*(Math.PI/180)/(TRACK/2);
      dt = dt*180/Math.PI; 
      
      //Clear tacho counts and put motors in freewheel mode. Then initialize tacho count
      oldTachoL = leftMotorTachoCount;
      oldTachoR = rightMotorTachoCount;
      odo.update(dx, dy, dt);
      
      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
