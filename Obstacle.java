/*Author: Michael S. Li
 *Date: 1/18/2015
 *Project: Arm Robot Collision Detector
 *Details: Code to calculate using conservative method (vMax, minDistance), the 
 *regions in the movement space that are guaranteed to be collision-free
 */

package CollisionDetectorProject;

//class that represents polygonal obstacles in the world; represented as an array of 
//smaller arrays of length 2 which store the x,y coordinates of the vertices in the polygon
public class Obstacle {
	private double[][] obstacle;
	
	//initializes new Obstacle object with an array of arrays of the vertices
	public Obstacle(double[][] vertices) {
		obstacle = new double[vertices.length][2];
		set(vertices);
	}

	//sets the vertices of the object with x,y coordinates
	public void set(double[][] vertices){
		for (int i = 0; i < obstacle.length; i++) {
			obstacle[i][0] = vertices[i][0];
			obstacle[i][1] = vertices[i][1];
		}	
	}
	
	//returns the obstacle represented by an array of array (with vertices)
	public double[][] get(){
		return obstacle;
	}
	
	//returns the number of vertices in the object
	public int getNumberOfVertices() {
		return obstacle.length;
	}
}