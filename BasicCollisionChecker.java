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
//of the arm robot and the obstacles in the world; very basic, only checks collision
//points at vertex of arms and assumes very conservative vMax at tip point at full extension
public class BasicCollisionChecker {
	private ArmRobot armRobot;
	private ArrayList<Obstacle> obstacles;
	private ArrayList<Line2D> obstacleLines;
	private ArrayList<Line2D> armRobotLines;
	static double THETA_V_MAX = Math.toRadians(5);
	private double vMax;
	
	//initializes the CollisionChecker object with parameters of the objects 
	//in the world (i.e., arm robot and arraylist of all the objects); also 
	//generates lines from obstacles, arm robot
	public BasicCollisionChecker(ArmRobot ar, ArrayList<Obstacle> obs) {
		armRobot = ar;
		obstacles = obs;
		vMax = THETA_V_MAX * (armRobot.getArm1Length() + 2*armRobot.getArm2Length());
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
	
	//returns the distance between the arm robot and the nearest obstacle
	public double getShortestDistance() {
		double shortestDistanceSq = 999999;
		
		for (Line2D armRobotLine: armRobotLines) {
			for (Line2D obstacleLine : obstacleLines) {
				double distance1 = armRobotLine.ptSegDistSq(obstacleLine.getX1(), obstacleLine.getY1());
				double distance2 = armRobotLine.ptSegDistSq(obstacleLine.getX2(), obstacleLine.getY2());
				double distance3 = obstacleLine.ptSegDistSq(armRobotLine.getX1(), armRobotLine.getY1());
				double distance4 = obstacleLine.ptSegDistSq(armRobotLine.getX2(), armRobotLine.getY2());
				
				if (Math.min(Math.min(distance1, distance2), Math.min(distance3, distance4)) < shortestDistanceSq) {
					shortestDistanceSq = Math.min(Math.min(distance1, distance2), Math.min(distance3, distance4));
				}
			}
		}
		
		return Math.sqrt(shortestDistanceSq);
	}
	
	//returns the minimum time needed to collide into obstacle given the shortest distance
	public double getTMin() {
		//System.out.println(getShortestDistance()/vMax);
		return (getShortestDistance()/vMax);
	}
	
	//returns the max angle movement you can be certain will be obstacle-free
	public double getMaxAngleMovement() {
		return (THETA_V_MAX * getTMin());	
	}
}
