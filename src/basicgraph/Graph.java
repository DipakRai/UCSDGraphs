package basicgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** An abstract class that implements a directed graph. 
 * The graph may have self-loops, parallel edges. 
 * Vertices are labeled by integers 0 .. n-1
 * and may also have String labels.
 * The edges of the graph are not labeled.
 * Representation of edges is left abstract.
 * 
 * @author UCSD MOOC development team and YOU
 * 
 */

public abstract class Graph {

	private int numVertices;
	private int numEdges;
	//optional association of String labels to vertices 
	private Map<Integer,String> vertexLabels;
	
	/**
	 * Create a new empty Graph
	 */
	public Graph() {
		numVertices = 0;
		numEdges = 0;
		vertexLabels = null;
	}

	
	/**
	 * Report size of vertex set
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return numVertices;
	}
	
	
	/**
	 * Report size of edge set
	 * @return The number of edges in the graph.
	 */	
	public int getNumEdges() {
		return numEdges;
	}
	
	/**
	 * Add new vertex to the graph.  This vertex will
	 * have as its index the next available integer.
	 * Precondition: contiguous integers are used to 
	 * index vertices.
	 * @return index of newly added vertex
	 */
	public int addVertex() {
		implementAddVertex();
		numVertices ++;
		return (numVertices-1);
	}
	
	/**
	 * Abstract method implementing adding a new
	 * vertex to the representation of the graph.
	 */
	public abstract void implementAddVertex();
	
	/**
	 * Add new edge to the graph between given vertices,
	 * @param u Index of the start point of the edge to be added. 
	 * @param v Index of the end point of the edge to be added. 
	 */
	public void addEdge(int v , int w) {
		numEdges ++;
		if (v < numVertices && w < numVertices) {
			implementAddEdge(v , w);			
		}
		else {
			throw new IndexOutOfBoundsException();
		}
	}
	
	/**
	 * Abstract method implementing adding a new
	 * edge to the representation of the graph.
	 */
	public abstract void implementAddEdge(int v, int w);
	
	/**
	 * Get all (out-)neighbors of a given vertex.
	 * @param v Index of vertex in question.
	 * @return List of indices of all vertices that are adjacent to v
	 * 	via outgoing edges from v. 
	 */
	public abstract List<Integer> getNeighbors(int v); 
	
	/**
	 * Get all in-neighbors of a given vertex.
	 * @param v Index of vertex in question.
	 * @return List of indices of all vertices that are adjacent to v
	 * 	via incoming edges to v. 
	 */
	public abstract List<Integer> getInNeighbors(int v);
	
	

	/** 
	 * The degree sequence of a graph is a sorted (organized in numerical order 
	 * from largest to smallest, possibly with repetitions) list of the degrees 
	 * of the vertices in the graph.
	 * 
	 * @return The degree sequence of this graph.
	 */
	public List<Integer> degreeSequence() {
		// XXX: Implement in part 1 of week 1
		//my implementation
		List <Integer> degreeSeqList = new ArrayList<Integer>();
		int numOfVertices = getNumVertices();
		for (int i=0; i < numOfVertices; i ++){
			degreeSeqList.add(getNeighbors(i).size() + getInNeighbors(i).size());
		}
		Collections.sort(degreeSeqList);
		Collections.reverse(degreeSeqList);
		return degreeSeqList;
	}
	
	/**
	 * Get all the vertices that are 2 away from the vertex in question.
	 * @param v The starting vertex
	 * @return A list of the vertices that can be reached in exactly two hops (by 
	 * following two edges) from vertex v.
	 * XXX: Implement in part 2 of week 1 for each subclass of Graph
	 */
	public abstract List<Integer> getDistance2(int v); 

	/** Return a String representation of the graph
	 * @return A string representation of the graph
	 */
	public String toString() {
		String s = "\nGraph with " + numVertices + " vertices and " + numEdges + " edges.\n";
		s += "Degree sequence: " + degreeSequence() + ".\n";
		if (numVertices <= 20) s += adjacencyString();
		return s;
	}

	/**
	 * Generate string representation of adjacency list
	 * @return the String
	 */
	public abstract String adjacencyString();

	
	// The next methods implement labeled vertices.
	// Basic graphs may or may not have labeled vertices.
	
	/**
	 * Create a new map of vertex indices to string labels
	 * (Optional: only if using labeled vertices.)
	 */
	public void initializeLabels() {
		vertexLabels = new HashMap<Integer,String>();
	}	
	/**
	 * Test whether some vertex in the graph is labeled 
	 * with a given index.
	 * @param The index being checked
	 * @return True if there's a vertex in the graph with this index; false otherwise.
	 */
	public boolean hasVertex(int v)
	{
		return v < getNumVertices();
	}
	
	/**
	 * Test whether some vertex in the graph is labeled 
	 * with a given String label
	 * @param The String label being checked
	 * @return True if there's a vertex in the graph with this label; false otherwise.
	 */
	public boolean hasVertex(String s)
	{
		return vertexLabels.containsValue(s);
	}
	
	/**
	 * Add label to an unlabeled vertex in the graph.
	 * @param The index of the vertex to be labeled.
	 * @param The label to be assigned to this vertex.
	 */
	public void addLabel(int v, String s) {
		if (v < getNumVertices() && !vertexLabels.containsKey(v)) 
		{
			vertexLabels.put(v, s);
		}
		else {
			System.out.println("ERROR: tried to label a vertex that is out of range or already labeled");
		}
	}
	
	/**
	 * Report label of vertex with given index
	 * @param The integer index of the vertex
	 * @return The String label of this vertex 
	 */
	public String getLabel(int v) {
		if (vertexLabels.containsKey(v)) {
			return vertexLabels.get(v);
		}
		else return null;
	}

	/**
	 * Report index of vertex with given label.
	 * (Assume distinct labels for vertices.)
	 * @param The String label of the vertex
	 * @return The integer index of this vertex 
	 */
	public int getIndex(String s) {
		for (Map.Entry<Integer,String> entry : vertexLabels.entrySet()) {
			if (entry.getValue().equals(s))
				return entry.getKey();
		}
		System.out.println("ERROR: No vertex with this label");
		return -1;
	}
	

	
	public static void main (String[] args) {
		//GraphLoader.createIntersectionsFile("data/maps/myucsd.map", "data/intersections/myucsd.intersections");
		// For testing of Part 1 functionality
		// Add your tests here to make sure your degreeSequence method is returning
		// the correct list, after examining the graphs.
		// my user DR test cases starts below
		
		GraphAdjList graphAdjList2 = new GraphAdjList();
		graphAdjList2.addVertex();
		graphAdjList2.addVertex();
		graphAdjList2.addVertex();
		graphAdjList2.addVertex();
		graphAdjList2.addVertex();
				
		graphAdjList2.addEdge(0, 1);
		graphAdjList2.addEdge(0, 2);
		graphAdjList2.addEdge(1, 3);
		graphAdjList2.addEdge(2, 3);
		graphAdjList2.addEdge(2, 1);
		graphAdjList2.addEdge(4, 3);
		
		System.out.println("Degree Sequence Test graphAdjList2 0 to 4 nodes="+ graphAdjList2.degreeSequence());
		
		System.out.println("Reassign 2 Hop Neighbor of vertex 0 graphAdjList2 0 to 4 nodes ="+graphAdjList2.getDistance2(0));
		System.out.println("Reassign 2 Hop Neighbor of vertex 1 ="+graphAdjList2.getDistance2(1));
		System.out.println("Reassign 2 Hop Neighbor of vertex 2 ="+graphAdjList2.getDistance2(2));
		System.out.println("Reassign 2 Hop Neighbor of vertex 3 ="+graphAdjList2.getDistance2(3));
		System.out.println("Reassign 2 Hop Neighbor of vertex 4 ="+graphAdjList2.getDistance2(4));
		
		GraphAdjMatrix graphAdjMatrix2 = new GraphAdjMatrix();
		graphAdjMatrix2.addVertex();
		graphAdjMatrix2.addVertex();
		graphAdjMatrix2.addVertex();
		graphAdjMatrix2.addVertex();
		graphAdjMatrix2.addVertex();
		graphAdjMatrix2.addEdge(0, 1);
		graphAdjMatrix2.addEdge(0, 2);
		graphAdjMatrix2.addEdge(1, 3);
		graphAdjMatrix2.addEdge(2, 3);
		graphAdjMatrix2.addEdge(2, 1);
		graphAdjMatrix2.addEdge(4, 3);
		
		System.out.println("Degree Sequence Test 0 to 4 nodes ="+ graphAdjMatrix2.degreeSequence());
		System.out.println("Reassign 2 Hop Neighbor of vertex 0 adjMatrix 0 to 4 nodes ="+graphAdjMatrix2.getDistance2(0));
		System.out.println("Reassign 2 Hop Neighbor of vertex 1 ="+graphAdjMatrix2.getDistance2(1));
		System.out.println("Reassign 2 Hop Neighbor of vertex 2 ="+graphAdjMatrix2.getDistance2(2));
		System.out.println("Reassign 2 Hop Neighbor of vertex 3 ="+graphAdjMatrix2.getDistance2(3));
		System.out.println("Reassign 2 Hop Neighbor of vertex 4 ="+graphAdjMatrix2.getDistance2(4));
		
		GraphAdjList graphAdjList = new GraphAdjList();
		graphAdjList.addVertex();
		graphAdjList.addVertex();
		graphAdjList.addVertex();
		graphAdjList.addVertex();
		//graphAdjList.addVertex();
				
		graphAdjList.addEdge(0, 1);
		graphAdjList.addEdge(1, 0);
		graphAdjList.addEdge(1, 2);
		graphAdjList.addEdge(2, 1);
		graphAdjList.addEdge(2, 3);
		graphAdjList.addEdge(3, 2);
		/*graphAdjList.addEdge(2, 3);
		graphAdjList.addEdge(2, 1);
		graphAdjList.addEdge(4, 3);*/
				
		System.out.println("Degree Sequence Test 0 to 3 nodes graphAdjList twoWays ="+ graphAdjList.degreeSequence());
		System.out.println("2 Hop Neighbor of vertex 0 graphAdjList 0 to 3 nodes only="+graphAdjList.getDistance2(0));
		System.out.println("2 Hop Neighbor of vertex 1 ="+graphAdjList.getDistance2(1));
		System.out.println("2 Hop Neighbor of vertex 2 ="+graphAdjList.getDistance2(2));
		System.out.println("2 Hop Neighbor of vertex 3 ="+graphAdjList.getDistance2(3));
		//System.out.println("2 Hop Neighbor of vertex 4 ="+graphAdjList.getDistance2(4));
	
		//System.out.println(" graphAdjList = "+graphAdjList.toString());
		//For testing Part 2 functionality
		// Test your distance2 code here.
		GraphAdjMatrix graphAdjMatrix = new GraphAdjMatrix();
		graphAdjMatrix.addVertex();
		graphAdjMatrix.addVertex();
		graphAdjMatrix.addVertex();
		graphAdjMatrix.addVertex();
		//graphAdjMatrix.addVertex();
		
		graphAdjMatrix.addEdge(0, 1);
		graphAdjMatrix.addEdge(1, 0);
		graphAdjMatrix.addEdge(1, 2);
		graphAdjMatrix.addEdge(2, 1);
		graphAdjMatrix.addEdge(2, 3);
		graphAdjMatrix.addEdge(3, 2);

		System.out.println("Degree Sequence Test 0 to 3 nodes adjMatrix twoWays ="+ graphAdjList.degreeSequence());
		System.out.println("2 Hop Neighbor of vertex 0 adjMatrix ="+graphAdjMatrix.getDistance2(0));
		System.out.println("2 Hop Neighbor of vertex 1 adjMatrix ="+graphAdjMatrix.getDistance2(1));
		System.out.println("2 Hop Neighbor of vertex 2 adjMatrix ="+graphAdjMatrix.getDistance2(2));
		System.out.println("2 Hop Neighbor of vertex 3 adjMatrix ="+graphAdjMatrix.getDistance2(3));
		//System.out.println("2 Hop Neighbor of vertex 4 adjMatrix ="+graphAdjMatrix.getDistance2(3));
					
		//System.out.println("Adj Matrix ="+graphAdjMatrix.adjacencyString());

/*		System.out.println("****");
		System.out.println("Loading graphs based on real data...");
		System.out.println("Goal: use degree sequence to analyse graphs.");
		
		System.out.println("Roads / intersections:");
		GraphAdjList graphFromFile = new GraphAdjList();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", graphFromFile);
		System.out.println(graphFromFile);
		
		System.out.println("Observe all degrees are <= 12.");
		System.out.println("****");

		System.out.println("\n****");
		
		// You can test with real road data here.  Use the data files in data/maps
		
		System.out.println("Flight data:");
		GraphAdjList airportGraph = new GraphAdjList();
		GraphLoader.loadRoutes("data/airports/routesUA.dat", airportGraph);
		System.out.println(airportGraph);
		System.out.println("Observe most degrees are small (1-30), eight are over 100.");
		System.out.println("****");
		
		System.out.println("Testing distance-two methods on sample graphs...");
		System.out.println("Goal: implement method using two approaches.");*/
		
	}
}
