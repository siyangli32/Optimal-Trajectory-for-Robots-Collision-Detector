/*Author: Michael S. Li
 *Date: 1/18/2015
 *Project: Arm Robot Collision Detector
 *Details: Code to calculate using conservative method (vMax, minDistance), the 
 *regions in the movement space that are guaranteed to be collision-free
 */

package CollisionDetectorProject;
import java.util.*;
import java.awt.geom.Line2D;

//class to test if there is collision and nearest distance to collision
//of the arm robot and the obstacles in the world; uses oracle function as well
//as point scan
public class OracleCollisionChecker {
	private ArmRobot armRobot;
	private ArrayList<Obstacle> obstacles;
	private ArrayList<Line2D> obstacleLines;
	private ArrayList<Line2D> armRobotLines;
	static double THETA_V_MAX = Math.toRadians(5);
	
	//initializes the CollisionChecker object with parameters of the objects 
	//in the world (i.e., arm robot and arraylist of all the objects); also 
	//generates lines from obstacles, arm robot
	public OracleCollisionChecker(ArmRobot ar, ArrayList<Obstacle> obs) {
		armRobot = ar;
		obstacles = obs;
		generateObstacleLines();
		generateArmRobotLines();
	}
	
	//breaks apart the obstacles in the world into an array of all the edge lines
	private void generateObstacleLines() {
		obstacleLines = new ArrayList<Line2D>();
		
		for (Obstacle ob : obstacles) {
			for (int vertex = 0; vertex < ob.getNumberOfVertices() - 1; vertex++) {
				Line2D newLine = new Line2D.Double(ob.get()[vertex][0], ob.get()[vertex][1], 
						ob.get()[vertex+1][0], ob.get()[vertex+1][1]);
				obstacleLines.add(newLine);
			}
			
			Line2D lastLine = new Line2D.Double(ob.get()[ob.getNumberOfVertices()-1][0], 
					ob.get()[ob.getNumberOfVertices()-1][1], ob.get()[0][0], ob.get()[0][1]);
			obstacleLines.add(lastLine);
		}
		/*add the edges of the world as obstacles
		Line2D bottomBorder = new Line2D.Double(0,0,windowWidth,0);
		Line2D leftBorder = new Line2D.Double(0,0,0,windowHeight);
		Line2D topBorder = new Line2D.Double(0,windowHeight,windowWidth,windowHeight);
		Line2D rightBorder = new Line2D.Double(windowWidth,0,windowWidth,windowHeight);
		
		obstacleLines.add(bottomBorder);
		obstacleLines.add(topBorder);
		obstacleLines.add(leftBorder);
		obstacleLines.add(rightBorder);
 		*/
	}
	
	//breaks apart the arm robot into a small array of the two edge lines
	private void generateArmRobotLines() {
		armRobotLines = new ArrayList<Line2D>();

		Line2D firstLine = new Line2D.Double(armRobot.getXYcoordinates()[0][0], 
				armRobot.getXYcoordinates()[0][1], armRobot.getXYcoordinates()[1][0], armRobot.getXYcoordinates()[1][1]);
		armRobotLines.add(firstLine);
		Line2D secondLine = new Line2D.Double(armRobot.getXYcoordinates()[1][0], 
				armRobot.getXYcoordinates()[1][1], armRobot.getXYcoordinates()[2][0], armRobot.getXYcoordinates()[2][1]);
		armRobotLines.add(secondLine);
	}
	
	//updates the lines in armRobotLines array with new vertex coordinates based on 
	//armRobot's current state
	public void updateArmRobotLines() {
		armRobotLines.get(0).setLine(armRobot.getXYcoordinates()[0][0], armRobot.getXYcoordinates()[0][1], 
				armRobot.getXYcoordinates()[1][0], armRobot.getXYcoordinates()[1][1]);
		armRobotLines.get(1).setLine(armRobot.getXYcoordinates()[1][0], armRobot.getXYcoordinates()[1][1], 
				armRobot.getXYcoordinates()[2][0], armRobot.getXYcoordinates()[2][1]);
	}
	
	//checks if there are any collisions between the arm robot and the obstacles
	//in the world
	public boolean isColliding() {
		boolean isColliding = false;
		
		for (Line2D armRobotLine : armRobotLines) {
			for (Line2D obstacleLine : obstacleLines) {
				if (armRobotLine.intersectsLine(obstacleLine)) {
					isColliding = true;
				}
			}
		}
		
		return isColliding;
	}
	
	//checks if the robot arm remains in the world's borders (note: only end-most vertex
	//has chance of going out of border)
	public boolean isInWorld(int windowWidth, int windowHeight) {
		boolean inWorld = true;
		
		for (int vertexNumber = 0; vertexNumber < armRobot.getXYcoordinates().length; vertexNumber++) {
			if ((armRobot.getXYcoordinates()[vertexNumber][0] < 0) || 
					(armRobot.getXYcoordinates()[vertexNumber][0] > windowWidth) ||
					(armRobot.getXYcoordinates()[vertexNumber][1] < 0) || 
					(armRobot.getXYcoordinates()[vertexNumber][1] > windowHeight)) {
				inWorld = false;
			}
		}
		
		return inWorld;	
	}
	
	//returns the distance between a point and the nearest obstacle
	public double getMinDistanceOfPoint(double[] xyCoordinates) {
		double shortestDistanceSq = 999999;
		double currentDistanceSq;
		
		for (Line2D obstacleLine : obstacleLines) {
			currentDistanceSq = obstacleLine.ptSegDistSq(xyCoordinates[0], xyCoordinates[1]);
			
			if (currentDistanceSq < shortestDistanceSq) {
				shortestDistanceSq = currentDistanceSq;
			}
		}
		
		return Math.sqrt(shortestDistanceSq);
	}
	
	//returns the minimum time needed to collide into obstacle by running a binary-search on
	//input times and the oracle function to test if that time allows for collision-free movement
	public double getTMin() {
		double testTime = 20;
		double timeDifference = testTime/2;
		boolean safe = false;
		
		while (timeDifference > .001 || safe == false) {
			safe = oracleFunction(testTime);
			if (safe) {
				testTime = testTime+timeDifference;
			}
			else {
				testTime = testTime-timeDifference;
			}
			timeDifference = timeDifference/2;
		}
		System.out.println("OracleCollisionChecker tMin: " + testTime);
		return testTime;
	}
	
	//returns the max angle movement you can be certain will be obstacle-free
	public double getMaxAngleMovement() {
		return (THETA_V_MAX * getTMin());	
	}
	
	//calculates the maximum velocity of a point along the arm robot using trigonometry
	//and vector velocities of both arms; can take in time to calculate max velocity 
	//if arm "straightened out" in that time
	public double calculatePointVMax(int distanceAlongArms, double time) {
		double vMax = 0;
		double adjustedArm2Angle = 0;

		if (armRobot.getArm2Angle() <= Math.PI) {
			adjustedArm2Angle = armRobot.getArm2Angle() - THETA_V_MAX * time;
			if (adjustedArm2Angle < 0) {
				adjustedArm2Angle = 0;
			}
		}
		if (armRobot.getArm2Angle() > Math.PI) {
			adjustedArm2Angle = armRobot.getArm2Angle() + THETA_V_MAX * time;
			if (adjustedArm2Angle > 2*Math.PI) {
				adjustedArm2Angle = 2*Math.PI;
			}
		}

		//System.out.println(time);
		//System.out.println("Angle: " + armRobot.getArm2Angle() + ", adjusted: " + adjustedArm2Angle);
		if (distanceAlongArms >= 0 && distanceAlongArms <= armRobot.getArm1Length()) {
			//System.out.println("First Arm");
			vMax = distanceAlongArms * THETA_V_MAX;
		}
		
		if (distanceAlongArms > armRobot.getArm1Length() && distanceAlongArms <= armRobot.getArm1Length() + armRobot.getArm2Length()) {
			double directDistanceToPoint = Math.sqrt(Math.pow(armRobot.getArm1Length(), 2) + Math.pow((distanceAlongArms-armRobot.getArm1Length()), 2) 
					- 2*(armRobot.getArm1Length())*(distanceAlongArms-armRobot.getArm1Length())*(Math.cos(Math.PI - adjustedArm2Angle)));
			double directAngleToPoint = Math.asin(((distanceAlongArms-armRobot.getArm1Length())/directDistanceToPoint)*Math.sin(Math.PI - adjustedArm2Angle));
			double arm1VelocityVectorAngle = Math.PI - armRobot.getArm1Angle() - directAngleToPoint;
			double arm2VelocityVectorAngle = armRobot.getArm1Angle() + adjustedArm2Angle + Math.PI;
			double arm1VelocityVectorMagnitude = directDistanceToPoint * THETA_V_MAX;
			double arm2VelocityVectorMagnitude = (distanceAlongArms-armRobot.getArm1Length()) * THETA_V_MAX;
			//System.out.println("Second Arm: " + directDistanceToPoint + ", " + directAngleToPoint + ", " + arm1VelocityVectorAngle + ", " + arm2VelocityVectorAngle + ", " + arm1VelocityVectorMagnitude+ ", " + arm2VelocityVectorMagnitude);
			double absoluteVelocity1 = Math.sqrt(Math.pow(Math.abs(Math.cos(arm1VelocityVectorAngle)*arm1VelocityVectorMagnitude) + Math.abs(Math.cos(arm2VelocityVectorAngle)*arm2VelocityVectorMagnitude), 2)
					+ Math.pow(Math.abs(Math.sin(arm1VelocityVectorAngle)*arm1VelocityVectorMagnitude) - Math.abs(Math.sin(arm2VelocityVectorAngle)*arm2VelocityVectorMagnitude), 2));
			double absoluteVelocity2 = Math.sqrt(Math.pow(Math.abs(Math.cos(arm1VelocityVectorAngle)*arm1VelocityVectorMagnitude) - Math.abs(Math.cos(arm2VelocityVectorAngle)*arm2VelocityVectorMagnitude), 2)
					+ Math.pow(Math.abs(Math.sin(arm1VelocityVectorAngle)*arm1VelocityVectorMagnitude) + Math.abs(Math.sin(arm2VelocityVectorAngle)*arm2VelocityVectorMagnitude), 2));
			//System.out.println("Absolute Velocity 1: " + absoluteVelocity1 + ", " + "Absolute Velocity 2: " + absoluteVelocity2);
			vMax = Math.max(absoluteVelocity1, absoluteVelocity2);
		}
		
		//System.out.println("Time: " + time + ", V: " + vMax + " Adjusted Angle: " + adjustedArm2Angle);
		
		return vMax;
	}
	
	//calculates the average maximum velocity if the arm straightened out in the time given to move
	public double calculateAveragePointVMax(int distanceAlongArms, double time) {
		//System.out.println(time);
		//System.out.println("Distance Along Arm: " + distanceAlongArms + ", Time: " + time + ", Unadjusted vMax: " + calculatePointVMax(distanceAlongArms, 0) + ", Adjusted vMax: " + calculatePointVMax(distanceAlongArms, time));
		return (calculatePointVMax(distanceAlongArms, 0) + calculatePointVMax(distanceAlongArms, time))/2;
	}
	
	//a test function that takes an input time and returns a boolean indicating if that movement time
	//would be guaranteed collision-free (true if collision-free)
	public boolean oracleFunction(double testTime) {
		for (int pointDistance = 0; pointDistance < armRobot.getArm1Length() + armRobot.getArm2Length(); pointDistance++) {
			double travelDistance = testTime * calculateAveragePointVMax(pointDistance, testTime);
			if (travelDistance >= getMinDistanceOfPoint(armRobot.getXYofPoint(pointDistance))) {
				//System.out.println(pointDistance);
				return false;
			}
		}
		return true;
	}
}

