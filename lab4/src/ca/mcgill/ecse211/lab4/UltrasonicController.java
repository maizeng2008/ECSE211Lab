package ca.mcgill.ecse211.lab4;

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();


	//int calGain(int distance);

	//int calGainForMin(int distance);
}
