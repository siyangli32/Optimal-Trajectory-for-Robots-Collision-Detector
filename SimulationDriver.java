/*Author: Michael S. Li
 *Date: 1/18/2015
 *Project: Arm Robot Collision Detector
 *Details: Code to calculate using conservative method (vMax, minDistance), the 
 *regions in the movement space that are guaranteed to be collision-free
 */

package CollisionDetectorProject;
import java.util.*;
import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.shape.ArcType;
import javafx.stage.Stage;

//driver that sets up the simulators with the arm robots and obstacles and employs
//the collision checker
public class SimulationDriver extends Application {
	private ArrayList<Obstacle> obstacleList;
	private ArmRobot armRobot;
	private BasicCollisionChecker collisionChecker1;
	private OracleCollisionChecker collisionChecker2;
	private OracleCollisionChecker2 collisionChecker3;
	private int numberOfTrials = 100;
	private static int WINDOW_WIDTH = 400;
	private static int WINDOW_HEIGHT = 400;
	private static int MAX_THETA1 = 90;
	private static int MAX_THETA2 = 360;
	private static int DISPLAY_SCALE_FACTOR = 3;
	
	//sets up the world, arm robots, and obstacles for simulation
	public void setUpSimulation() {
		armRobot = new ArmRobot(250, 150, .7853, 0);
		obstacleList = new ArrayList<Obstacle>(3);
		
		double[][] vertices1 = {{50,300}, {50,350}, {100,350}};
		Obstacle obstacle1 = new Obstacle(vertices1);
		double[][] vertices2 = {{330,120}, {380,120}, {380,170}, {330,170}};
		Obstacle obstacle2 = new Obstacle(vertices2);
		double[][] vertices3 = {{250,300}, {270, 270}, {240,195}, {220, 220}, {210,280}};
		Obstacle obstacle3 = new Obstacle(vertices3);
		
		obstacleList.add(obstacle1);
		obstacleList.add(obstacle2);
		obstacleList.add(obstacle3);
		
		collisionChecker1 = new BasicCollisionChecker(armRobot, obstacleList);
		collisionChecker2 = new OracleCollisionChecker(armRobot, obstacleList);
		collisionChecker3 = new OracleCollisionChecker2(armRobot, obstacleList);
	}
	
	//performs simulations using CollisionChekcer object and gets resulting data array from simulations
	public ArrayList<double[]> getData(int totalSamples) {
		//creates a Random object to generate random angles for our mapping of the movement space
		ArrayList<double[]> dataArray = new ArrayList<double[]>();
		final Random randomGenerator = new Random();
		double newAngle1;
		double newAngle2;
		
		//for-loop which samples many random initial configurations and uses collisionChecker
		//to find the minimum distance from collision for those initial configurations which
		//are valid (do not collide and are in the bounds of the world)
		for (int sampleNumber = 0; sampleNumber < totalSamples; sampleNumber++) {
			newAngle1 = randomGenerator.nextFloat()*MAX_THETA1;
			newAngle2 = randomGenerator.nextFloat()*MAX_THETA2;
			armRobot.changeAngle1(Math.toRadians(newAngle1));
			armRobot.changeAngle2(Math.toRadians(newAngle2));
			collisionChecker1.updateArmRobotLines();
			collisionChecker2.updateArmRobotLines();
			collisionChecker3.updateArmRobotLines();
			
			if ((! collisionChecker1.isColliding()) && (collisionChecker1.isInWorld(WINDOW_WIDTH, WINDOW_HEIGHT))) {
				double[] sampleData = new double[11];
				sampleData[0] = newAngle1;
				sampleData[1] = newAngle2;
				sampleData[2] = Math.toDegrees(collisionChecker1.getMaxAngleMovement());
				sampleData[3] = armRobot.getXYcoordinates()[0][0];
				sampleData[4] = armRobot.getXYcoordinates()[0][1];
				sampleData[5] = armRobot.getXYcoordinates()[1][0];
				sampleData[6] = armRobot.getXYcoordinates()[1][1];
				sampleData[7] = armRobot.getXYcoordinates()[2][0];
				sampleData[8] = armRobot.getXYcoordinates()[2][1];
				sampleData[9] = Math.toDegrees(collisionChecker2.getMaxAngleMovement());
				sampleData[10] = Math.toDegrees(collisionChecker3.getMaxAngleMovement());
				dataArray.add(sampleData);
				
				/*
				System.out.println("------------------------------");
				System.out.println("Theta1: " + Math.toDegrees(newAngle1) + " Theta2: " + Math.toDegrees(newAngle2));
				System.out.println("AX: " + armRobot.getXYcoordinates()[0][0] + " AY: " + armRobot.getXYcoordinates()[0][1] + " BX: " + armRobot.getXYcoordinates()[1][0] + " BY: " + armRobot.getXYcoordinates()[1][1] + " CX: " + armRobot.getXYcoordinates()[2][0] + " CY: " + armRobot.getXYcoordinates()[2][1]);
				System.out.println("Shortest Distance: " + collisionChecker.getShortestDistance());
				System.out.println("Minimum Time to Collision: " + collisionChecker.getTMin());
				System.out.println("Maximum Angle Movement: " + Math.toDegrees(collisionChecker.getMaxAngleMovement()));
				*/
			}
		}
		
		return dataArray;	
	}
	
	//runs through the points in the configuration space (by increment of 1 degrees)
	//and returns a array of arrays (90 by 360) of ints; 1 if that configuration
	//is a collision with obstacle; 2 if that configuration is out of the bounds 
	//of the world; 0 if else
	public int[][] getCollidingData() {
		int[][] collidingData = new int[MAX_THETA1+1][MAX_THETA2+1];
		
		for (int theta1 = 0; theta1 <= MAX_THETA1; theta1++) {
			for (int theta2 = 0; theta2 <= MAX_THETA2; theta2++) {
				armRobot.changeAngle1(Math.toRadians(theta1));
				armRobot.changeAngle2(Math.toRadians(theta2));
				collisionChecker1.updateArmRobotLines();;
				if (collisionChecker1.isColliding()) {
					collidingData[theta1][theta2] = 1;
				}
				else {
					collidingData[theta1][theta2] = 0;
				}
				if (!collisionChecker1.isInWorld(WINDOW_HEIGHT, WINDOW_WIDTH)) {
					collidingData[theta1][theta2] = 2;
				}
			}
		}
		
		return collidingData;	
	}
	
	//code to draw and display resulting data onto canvas; code from Oracle website on
	//JavaFx API used as basic template
    public void start(Stage primaryStage) {
        primaryStage.setTitle("Collision Detector Project");
        Group root = new Group();
        
        Canvas canvas = new Canvas(MAX_THETA2*DISPLAY_SCALE_FACTOR, MAX_THETA1*DISPLAY_SCALE_FACTOR);
        Canvas canvas2 = new Canvas(MAX_THETA2*DISPLAY_SCALE_FACTOR, WINDOW_HEIGHT+MAX_THETA1*DISPLAY_SCALE_FACTOR+20);
        GraphicsContext gc = canvas.getGraphicsContext2D();
        GraphicsContext gc2 = canvas2.getGraphicsContext2D();
        
        setUpSimulation();
        int[][] collisionData = getCollidingData();
        ArrayList<double[]> dataArray = getData(numberOfTrials);
        
        drawMovementSpace(gc, dataArray, collisionData);
        drawSimulationSpace(gc2, dataArray);
        
        root.getChildren().add(canvas);
        root.getChildren().add(canvas2);
        
        primaryStage.setScene(new Scene(root));
        primaryStage.show();
    }
    
    //draws the collision-free regions in the configuration space based on initial configuration
    //and maximum angle movement certain to be collision-free
    private void drawMovementSpace(GraphicsContext gc, ArrayList<double[]> dataArray, int[][] collidingData) {
        gc.setStroke(Color.BLACK);
        gc.setLineWidth(3);
        
        gc.strokeLine(0, 0, MAX_THETA2*DISPLAY_SCALE_FACTOR, 0);
        gc.strokeLine(0, MAX_THETA1*DISPLAY_SCALE_FACTOR, MAX_THETA2*DISPLAY_SCALE_FACTOR, MAX_THETA1*DISPLAY_SCALE_FACTOR);
        gc.strokeLine(0, 0, 0, MAX_THETA1*DISPLAY_SCALE_FACTOR);
        gc.strokeLine(MAX_THETA2*DISPLAY_SCALE_FACTOR, 0, MAX_THETA2*DISPLAY_SCALE_FACTOR, MAX_THETA1*DISPLAY_SCALE_FACTOR);
        
        gc.setLineWidth(1);
        
        for (int theta1 = 0; theta1 <= MAX_THETA1; theta1++) {
        	for (int theta2 = 0; theta2 <= MAX_THETA2; theta2++) {
        		if (collidingData[theta1][theta2] == 1) {
        			gc.fillOval(theta2*DISPLAY_SCALE_FACTOR, theta1*DISPLAY_SCALE_FACTOR, 1, 2);
        		}
        		if (collidingData[theta1][theta2] == 2) {
        			gc.fillOval(theta2*DISPLAY_SCALE_FACTOR, theta1*DISPLAY_SCALE_FACTOR, 1, 1);
        		}
        	} 	
        }
        
        for (double[] sampleData : dataArray) {
        	gc.strokeRect(sampleData[1]*DISPLAY_SCALE_FACTOR-(sampleData[2]*DISPLAY_SCALE_FACTOR/2), 
        			sampleData[0]*DISPLAY_SCALE_FACTOR-(sampleData[2]*DISPLAY_SCALE_FACTOR/2), 
        			sampleData[2]*DISPLAY_SCALE_FACTOR, sampleData[2]*DISPLAY_SCALE_FACTOR);
        }
        
        gc.setStroke(Color.DARKBLUE);
        for (double[] sampleData : dataArray) {
        	gc.strokeRect(sampleData[1]*DISPLAY_SCALE_FACTOR-(sampleData[9]*DISPLAY_SCALE_FACTOR/2), 
        			sampleData[0]*DISPLAY_SCALE_FACTOR-(sampleData[9]*DISPLAY_SCALE_FACTOR/2), 
        			sampleData[9]*DISPLAY_SCALE_FACTOR, sampleData[9]*DISPLAY_SCALE_FACTOR);
        }
        
        gc.setStroke(Color.DARKGREEN);
        for (double[] sampleData : dataArray) {
        	gc.strokeRect(sampleData[1]*DISPLAY_SCALE_FACTOR-(sampleData[10]*DISPLAY_SCALE_FACTOR/2), 
        			sampleData[0]*DISPLAY_SCALE_FACTOR-(sampleData[10]*DISPLAY_SCALE_FACTOR/2), 
        			sampleData[10]*DISPLAY_SCALE_FACTOR, sampleData[10]*DISPLAY_SCALE_FACTOR);
        }
    }
    
    //draws the arm movement in the simulation space over all the sample trials
    private void drawSimulationSpace(GraphicsContext gc, ArrayList<double[]> dataArray) {
    	gc.setFill(Color.DARKBLUE);
    	gc.setStroke(Color.BLACK);
        gc.setLineWidth(2);
        final int LEFT_SHIFT = (MAX_THETA2*DISPLAY_SCALE_FACTOR-WINDOW_WIDTH)/2;
        final int DOWN_SHIFT = MAX_THETA1*DISPLAY_SCALE_FACTOR+10;
        
        gc.strokeLine(0+LEFT_SHIFT, 0+DOWN_SHIFT, WINDOW_WIDTH+LEFT_SHIFT, 0+DOWN_SHIFT);
        gc.strokeLine(0+LEFT_SHIFT, WINDOW_HEIGHT+DOWN_SHIFT, WINDOW_WIDTH+LEFT_SHIFT, WINDOW_HEIGHT+DOWN_SHIFT);
        gc.strokeLine(0+LEFT_SHIFT, 0+DOWN_SHIFT, 0+LEFT_SHIFT, WINDOW_HEIGHT+DOWN_SHIFT);
        gc.strokeLine(WINDOW_WIDTH+LEFT_SHIFT, 0+DOWN_SHIFT, WINDOW_WIDTH+LEFT_SHIFT, WINDOW_HEIGHT+DOWN_SHIFT);
        
        gc.setStroke(Color.GREY);
        gc.setLineWidth(1);
        for (double[] sample : dataArray) {
        	gc.strokeLine(sample[3]+LEFT_SHIFT, sample[4]+DOWN_SHIFT, sample[5]+LEFT_SHIFT, sample[6]+DOWN_SHIFT);
        	gc.strokeLine(sample[5]+LEFT_SHIFT, sample[6]+DOWN_SHIFT, sample[7]+LEFT_SHIFT, sample[8]+DOWN_SHIFT);
        }
        
        for (Obstacle obstacle : obstacleList) {
        	double[] xCoordinates = new double[obstacle.getNumberOfVertices()];
        	double[] yCoordinates = new double[obstacle.getNumberOfVertices()];
        	
        	for (int vertex = 0; vertex < obstacle.getNumberOfVertices(); vertex++) {
        		xCoordinates[vertex] = obstacle.get()[vertex][0] + LEFT_SHIFT;
        		yCoordinates[vertex] = obstacle.get()[vertex][1] + DOWN_SHIFT;
        	}
        	
        	gc.fillPolygon(xCoordinates, yCoordinates, obstacle.getNumberOfVertices());
        }  
    }
	
	public static void main(String[] args) {
        launch(args);
    }
 
}