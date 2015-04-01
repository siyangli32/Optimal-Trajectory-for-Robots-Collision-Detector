/*Author: Michael S. Li
 *Date: 1/18/2015
 *Project: Arm Robot Collision Detector
 *Details: Code to calculate using conservative method (vMax, minDistance), the 
 *regions in the movement space that are guaranteed to be collision-free
 */

package CollisionDetectorProject;

//class to represent one-dimensional two segment arm robot with fields storing
//arm length and angle
public class ArmRobot {
	private int arm1Length;
	private int arm2Length;
	private double arm1Angle;
	private double arm2Angle;
	private static int INITIAL_AX = 0;
	private static int INITIAL_AY = 0;

	//initializes the ArmRobot object with length of arms and initial angles
	public ArmRobot(int al1, int al2, double theta1, double theta2) {
		arm1Length = al1;
		arm2Length = al2;
		arm1Angle = theta1;
		arm2Angle = theta2;
	}
	
	//using trigonometric calculations to obtain x and y coordinates of the vertices
	//from segment lengths and angles; return as an array of smaller arrays of length 2 (to
	//store x,y coordinates)
	public double[][] getXYcoordinates() {
		double[][] xy_arm_coordinates = new double[3][2]; 
		double pointAX = INITIAL_AX;
		double pointAY = INITIAL_AY;
		double pointBX;
		double pointBY;
		double pointCX;
		double pointCY;
		
		pointBX = Math.cos(arm1Angle)*arm1Length;
		pointBY = Math.sin(arm1Angle)*arm1Length;
		
		pointCX = Math.cos(arm1Angle + arm2Angle)*arm2Length + pointBX;
		pointCY = Math.sin(arm1Angle + arm2Angle)*arm2Length + pointBY;
		
		xy_arm_coordinates[0][0] = pointAX;
		xy_arm_coordinates[0][1] = pointAY;
		xy_arm_coordinates[1][0] = pointBX;
		xy_arm_coordinates[1][1] = pointBY;
		xy_arm_coordinates[2][0] = pointCX;
		xy_arm_coordinates[2][1] = pointCY;
 		
		return xy_arm_coordinates;	
	}
	
	//change angle 1
	public void changeAngle1(double newAngle1) {
		arm1Angle = newAngle1;
	}
	
	//change angle 2
	public void changeAngle2(double newAngle2) {
		arm2Angle = newAngle2;
	}
	
	//returns the arm 1's length
	public int getArm1Length() {
		return arm1Length;
	}
	
	//returns the arm 2's length
	public int getArm2Length() {
		return arm2Length;
	}
	
	//returns the arm 1's angle
	public double getArm1Angle() {
		return arm1Angle;
	}
	
	//returns the arm 2's angle
	public double getArm2Angle() {
		return arm2Angle;
	}
	
	//returns the x,y coordinates of a point on the arm
	public double[] getXYofPoint(int distanceAlongArms) {
		if (distanceAlongArms >= 0 && distanceAlongArms <= arm1Length) {
			double pointX = Math.cos(arm1Angle)*distanceAlongArms;
			double pointY = Math.sin(arm1Angle)*distanceAlongArms;
			double[] pointXY = {pointX, pointY};
			return pointXY;
		}
		
		if (distanceAlongArms > arm1Length && distanceAlongArms <= arm1Length + arm2Length) {
			double pointX = Math.cos(arm1Angle + arm2Angle)*(distanceAlongArms - arm1Length) + Math.cos(arm1Angle)*arm1Length;
			double pointY = Math.sin(arm1Angle + arm2Angle)*(distanceAlongArms - arm1Length) + Math.sin(arm1Angle)*arm2Length;
			double[] pointXY = {pointX, pointY};
			return pointXY;
		}
		
		else {
			return null;
		}
	}
}
