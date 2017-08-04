/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

public class MapGraph {
	
	static int dijkstraCount ;
	static int aStarCount ;
	
	private HashMap <GeographicPoint, MapNode> mapGraphVertices;
	
	/**
	 * @return the mapGraphVertices
	 */
	public HashMap <GeographicPoint, MapNode> getMapGraphVertices() {
		return mapGraphVertices;
	}

	/**
	 * @param mapGraphVertices the mapGraphVertices to set
	 */
	public void setMapGraphVertices(HashMap <GeographicPoint, MapNode> mapGraphVertices) {
		this.mapGraphVertices = mapGraphVertices;
	}

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		mapGraphVertices = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {

		return mapGraphVertices.values().size();
		
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return mapGraphVertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
				
		int totalNumEdges =0;
		
		Iterator<MapNode> iterator = mapGraphVertices.values().iterator();
		while (iterator.hasNext()){
			MapNode mapNode = iterator.next();
			//System.out.println(mapNode.getLocation());
			//System.out.println("hey edges ="+mapNode.getEdges());
			if(null!=mapNode.getEdges()){
				//System.out.println("size="+mapNode.getEdges().size());
				totalNumEdges += mapNode.getEdges().size();
			} else {
				totalNumEdges +=0;
			}
		}
		return totalNumEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		boolean addVertexStatus = false;
		if (null!=location && !mapGraphVertices.containsKey(location)){
			MapNode mapNode = new MapNode();
			mapNode.setLocation(location);
			mapGraphVertices.put(location, mapNode);
			addVertexStatus=true;
		} else {
			addVertexStatus=false;
		}
		return addVertexStatus;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		MapEdge edge = new MapEdge();
		if (null != from && null != to && null != roadName && null != roadType) {
			edge.setStartLatLong(from);
			edge.setEndLatLong(to);
			edge.setRoadName(roadName);
			edge.setRoadType(roadType);
			edge.setLength(length);
			if (mapGraphVertices.containsKey(from)) {
				Set<MapEdge> edges = mapGraphVertices.get(from).getEdges();
				if (null!=edges){
					edges.add(edge);
				} else {
					edges = new HashSet<MapEdge>();
					edges.add(edge);
				}
				mapGraphVertices.get(from).setEdges(edges);
			}
		}
	}
	
	/**
	 * The method returns the neighbors of the location passed as a param
	 * @author dipakrai 
	 * @param location
	 * @return neighborNodes
	 */
	public List<GeographicPoint> getNeighbors(GeographicPoint location) {
		List <GeographicPoint> neighborGeoLocs = new ArrayList<GeographicPoint>();
		if (mapGraphVertices.containsKey(location)){
			if (null!=mapGraphVertices.get(location).getEdges()&& mapGraphVertices.get(location).getEdges().size()>0){
				for (MapEdge mapEdge : mapGraphVertices.get(location).getEdges()) {
					neighborGeoLocs.add(mapEdge.getEndLatLong());
				}
			}
		}
		return neighborGeoLocs;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        //System.out.println("calling bfs now");
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)	{
		
		if (null != start && null != goal && mapGraphVertices.containsKey(start)
				&& mapGraphVertices.containsKey(goal)) {
		Map <GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		Set	<GeographicPoint> visitedNodeSet = new HashSet<GeographicPoint>();

		LinkedList<GeographicPoint> nodeQueueToExplore = new LinkedList<GeographicPoint>();
		List<GeographicPoint> neighborGeoLocs = new ArrayList<GeographicPoint>();
		//add start to queue
		nodeQueueToExplore.addLast(start);
		//adding start to visited nodes
		visitedNodeSet.add(start);	
		GeographicPoint currentLocation = null;
		
		while (!nodeQueueToExplore.isEmpty()){
			//queue loop starts
			//initialize currentLocation to start
			currentLocation = nodeQueueToExplore.poll();
			// Hook for visualization.
			nodeSearched.accept(currentLocation);
			if(currentLocation.equals(goal)){
				//System.out.println("i m going to break;");
				break;
			}
			//getting neighbors of currentLocation
			neighborGeoLocs = getNeighbors(currentLocation);
			//System.out.println(" Let's print the neigbors of "+ currentLocation);
			//for (GeographicPoint geographicPoint : neighborGeoLocs) {
				//System.out.println(geographicPoint.getX() + " " + geographicPoint.getY());
			//}

			//check if neighborNode n is not in the visitedSet then add n to queue
			for (GeographicPoint geographicPointNeighbor : neighborGeoLocs) {
				if(!visitedNodeSet.contains(geographicPointNeighbor)){
					// add new neighbor to visitedSet
					visitedNodeSet.add(geographicPointNeighbor);
					// add currentNode as new neighbor's parent in parentMap
					//System.out.println("Adding neighbor "+geographicPointNeighbor + " to parentMap");
					parentMap.put(geographicPointNeighbor, currentLocation);
					//enqueue new neighbor to the queue
					nodeQueueToExplore.addLast(geographicPointNeighbor);
				}
			}
			//System.out.println("reached the end of the queue loop....");
			/*for(Entry<GeographicPoint, GeographicPoint> geoPoint : parentMap.entrySet()){
				System.out.println(geoPoint.getKey() + " "+ geoPoint.getValue());
			}	*/		
			//queue loop ends
		}
		
		if (!currentLocation.equals(goal)){
			//System.out.println("returning null");
			return null;
		}
		//System.out.println("currentLocation!=goal="+(currentLocation!=goal));
		/*for(Entry<GeographicPoint, GeographicPoint> entry: parentMap.entrySet()){
			System.out.println("entry key ="+entry.getKey() + " entry value ="+entry.getValue());
		}*/
		 return buildRoutePath(parentMap, goal, start);
		} else {
			return null;
		}
	}
	/**
	 * This method searches for the shortest route on the bfs algorithm
	 * @param parentMapReceived
	 * @param goal
	 * @param start
	 * @return List <GeographicPoint>
	 */
	private List<GeographicPoint> buildRoutePath (Map<GeographicPoint, GeographicPoint> parentMapReceived,GeographicPoint goal, GeographicPoint start){
		//preparing a linked list of geographic locations
		//System.out.println("Did i reach here..");
		List <GeographicPoint> bfsRouteGeoLocList = new LinkedList <GeographicPoint>();
		//add goal to the route
		bfsRouteGeoLocList.add(goal);
		boolean startGeoLocFound = false;
		// setting the current location as the destination location to trace back to the source location
		GeographicPoint currentGeoLoc = goal;
		//System.out.println("startGeoLocFound ="+startGeoLocFound);
		//run the loop until the start location is found in the parentMapReceived
		while (startGeoLocFound != true) {
			//System.out.println(" I enter the dragon");
			//iterating through each of the elements of the parentMap
			for (Entry<GeographicPoint, GeographicPoint> entry : parentMapReceived.entrySet()) {
				//System.out.println("Enter the map...");
				// the current location is exactly whose parent I am tracing back right upto the start location
				if (currentGeoLoc.equals(entry.getKey())) {
					//System.out.println("TRUE entry.getKey() " + entry.getKey() + " currentGeoLoc ="+ currentGeoLoc );
					//setting the parent found as the current location to trace it's parent in next iteration of for loop
					currentGeoLoc = entry.getValue();
					//adding the parent found to the route list
					bfsRouteGeoLocList.add(currentGeoLoc);
				}
				if (currentGeoLoc.equals(start)) {
					// the current location is actually the start location so hurrah break
					//System.out.println(" currentGeoLoc == start =" + currentGeoLoc + " start =" +start);
					startGeoLocFound = true;
				}
				if(startGeoLocFound) break;
			}
		}
		//System.out.println(bfsRouteGeoLocList);
		Collections.reverse(bfsRouteGeoLocList);
		return bfsRouteGeoLocList;
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		
		Set<MapNode> visitedNodeSet = new HashSet<MapNode>();
		Map<MapNode, MapNode> parentMap =  new HashMap<MapNode, MapNode>();
		MapNode currentNode = null;	
		
		PriorityQueue<MapNode> pQueue =  new PriorityQueue<MapNode>();
		if (null!= start && null!= goal && mapGraphVertices.containsKey(start) && mapGraphVertices.containsKey(goal)){
			//System.out.println("Hey i entered here.....");
			MapNode startNode = mapGraphVertices.get(start);
			startNode.setActualDurFrmStart(0);
			//System.out.println(" startNode = "+startNode.getLocation());
			pQueue.add(startNode);
			//System.out.println(" pQueue "+pQueue.size());
			dijkstraCount=0;
			while (!pQueue.isEmpty()) {				
				currentNode=pQueue.poll();
				dijkstraCount++;
				nodeSearched.accept(currentNode.getLocation());
			if (!visitedNodeSet.contains(currentNode)) {
				visitedNodeSet.add(currentNode);
				if (currentNode.getLocation().equals(goal)) {
					//System.out.println("!@$!@$@! Hurrah curentNode= "+currentNode.getLocation() + " goal ="+goal.getX()+" "+goal.getY());
					break;
				}
				
				//System.out.println("getting neighbors of ="+currentNode.getLocation());
				List <GeographicPoint> neighborGeoLocs = getNeighbors(currentNode.getLocation());
				//System.out.println("the neighbors of "+ currentNode.getLocation() + " are = "+neighborGeoLocs);
				for (GeographicPoint neighborGeoPoint : neighborGeoLocs) {
					MapNode neighborNode = mapGraphVertices.get(neighborGeoPoint);
					Set<MapEdge> edgeSet = currentNode.getEdges();
					MapEdge edge = null;
					double edgeDuration=0;
					Iterator <MapEdge> edgeIterator = edgeSet.iterator();
					while (edgeIterator.hasNext()) {
						edge = new MapEdge();
						edge = edgeIterator.next();
						//System.out.println("neighborNode.getLocation() ="+ neighborNode.getLocation() + " edge.getEndLatLong() ="+edge.getEndLatLong());
							if (neighborNode.getLocation().equals(edge.getEndLatLong())) {
							//System.out.println("Found the edge whose startpoint is the currentNode "+ currentNode.getLocation());
							//System.out.println("Edge start point and end point are "+edge.getStartLatLong() + " "+ edge.getEndLatLong());
							edgeDuration =0;
							edgeDuration = getTimeForThisEdge(edge);
							//System.out.println(" Am I infinity neighborNode= "+ neighborNode.getActualDistFrmStart());
							//System.out.println(" Am I infinity currentNode= " + currentNode.getActualDistFrmStart());
								if (Double.POSITIVE_INFINITY!=neighborNode.getActualDurFrmStart()) {
									double tempDurFrmStart = currentNode.getActualDurFrmStart() + edgeDuration;
									//System.out.println( "tempDistFrmStart ="+tempDistFrmStart);
										if (neighborNode.getActualDurFrmStart() > tempDurFrmStart){
											//System.out.println( "tempDistFrmStart ="+tempDistFrmStart);
											//System.out.println("neighborNode.getActualDistFrmStart() > tempDistFrmStart ="+(neighborNode.getActualDistFrmStart() > tempDistFrmStart));
											neighborNode.setActualDurFrmStart(tempDurFrmStart);
											//System.out.println( "Adding "+ currentNode + " as parent of " + neighborNode);
											parentMap.put(neighborNode, currentNode);
										}
								} else {
									double tempDistFrmStart = currentNode.getActualDurFrmStart() + edgeDuration;
					//				System.out.println("actual Dist of neighborNode to be updated since not infinity is "+neighborNode + " is "+ tempDistFrmStart);
									neighborNode.setActualDurFrmStart(tempDistFrmStart);
						//			System.out.println("neighborNode.getActualDistFrmStart() > tempDistFrmStart ="+(neighborNode.getActualDistFrmStart() > tempDistFrmStart));
									neighborNode.setActualDurFrmStart(tempDistFrmStart);
							//		System.out.println( "Adding "+ currentNode + " as parent of " + neighborNode);
									parentMap.put(neighborNode, currentNode);
								}
							}
					}
					pQueue.add(neighborNode);
				}
			}
		}		
			if (!currentNode.getLocation().equals(goal)){
				return null;
			}				
				System.out.println("going to call buildDijkstra....");
				return buildDijkstraRoutePath(parentMap,goal,start);
		} else {
				return null;
			}
	}
	
	private double getTimeForThisEdge(MapEdge edge){
		
		return (edge.getLength()/getEdgeSpeed(edge));
	}
	
	private double getEdgeSpeed(MapEdge edge){
		
		double speed = 0;
		if (edge.getRoadType().equalsIgnoreCase("MOTORWAY")||edge.getRoadType().equalsIgnoreCase("TRUNK")||edge.getRoadType().equalsIgnoreCase("PRIMARY")){
			speed = RoadSpeedConstant.MOTORWAY; 
		} else if (edge.getRoadType().equalsIgnoreCase("RESIDENTIAL")||edge.getRoadType().equalsIgnoreCase("TERTIARY") ||edge.getRoadType().equalsIgnoreCase("SECONDARY") ) {
				speed = RoadSpeedConstant.TERTIARY;
		} else {
			speed = RoadSpeedConstant.UNCLASSIFIED;
		}
		return speed;
	}
	
	private List <GeographicPoint> buildDijkstraRoutePath(Map<MapNode, MapNode>parentMap, GeographicPoint goal, GeographicPoint start){
				//preparing a linked list of geographic locations
				//System.out.println("Did i reach here buildDijkstraRoutePath..");
				List <GeographicPoint> djikstraGeoLocList = new LinkedList <GeographicPoint>();
				//add goal to the route
				djikstraGeoLocList.add(goal);
				boolean startGeoLocFound = false;
				// setting the current location as the destination location to trace back to the source location
				GeographicPoint currentGeoLoc = goal;
				//System.out.println("buildDijkstraRoutePath startGeoLocFound ="+startGeoLocFound);
				//System.out.println(" buildDijkstraRoutePath parentMap size = "+parentMap.entrySet().size());
				//run the loop until the start location is found in the parentMapReceived
				while (startGeoLocFound != true) {
					//System.out.println(" I enter the dragon");
					//iterating through each of the elements of the parentMap
					
					for (Entry<MapNode, MapNode> entry : parentMap.entrySet()) {
						//System.out.println("Enter the map...");
						// the current location is exactly whose parent I am tracing back right upto the start location
						if (currentGeoLoc.equals(entry.getKey().getLocation())) {
							//System.out.println("TRUE entry.getKey() " + entry.getKey() + " currentGeoLoc ="+ currentGeoLoc );
							//setting the parent found as the current location to trace it's parent in next iteration of for loop
							currentGeoLoc = entry.getValue().getLocation();
							//adding the parent found to the route list
							djikstraGeoLocList.add(currentGeoLoc);
						}
						if (currentGeoLoc.equals(start)) {
							// the current location is actually the start location so hurrah break
					//		System.out.println(" currentGeoLoc == start =" + currentGeoLoc + " start =" +start);
							startGeoLocFound = true;
						}
						if(startGeoLocFound) break;
					}
				}
				//System.out.println(djikstraGeoLocList);
				Collections.reverse(djikstraGeoLocList);
				return djikstraGeoLocList;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		
		Set<MapNode> visitedNodeSet = new HashSet<MapNode>();
		Map<MapNode, MapNode> parentMap =  new HashMap<MapNode, MapNode>();
		MapNode currentNode = null;	
		
		PriorityQueue<MapNode> pQueue =  new PriorityQueue<MapNode>();
		if (null!= start && null!= goal && mapGraphVertices.containsKey(start) && mapGraphVertices.containsKey(goal)){
			//System.out.println("Hey i entered here.....");
			MapNode startNode = mapGraphVertices.get(start);
			startNode.setActualDurFrmStart(0);
			//System.out.println(" startNode = "+startNode.getLocation());
			pQueue.add(startNode);
			//System.out.println(" pQueue "+pQueue.size());
			aStarCount=0;
			while (!pQueue.isEmpty()) {
				
				currentNode=pQueue.poll();
				aStarCount++;
				
				nodeSearched.accept(currentNode.getLocation());
				//System.out.println("nodeSearched="+nodeSearched);
				//System.out.println("currentNode ="+currentNode.toString());
			if (!visitedNodeSet.contains(currentNode)) {
				//System.out.println("VisitedNodeSet did not contain ="+currentNode);
				visitedNodeSet.add(currentNode);
				if (currentNode.getLocation().equals(goal)) {
					//System.out.println("!@$!@$@! Hurrah curentNode= "+currentNode.getLocation() + " goal ="+goal.getX()+" "+goal.getY());
					break;
				}
				
				//System.out.println("getting neighbors of ="+currentNode.getLocation());
				List <GeographicPoint> neighborGeoLocs = getNeighbors(currentNode.getLocation());
				//System.out.println("the neighbors of "+ currentNode.getLocation() + " are = "+neighborGeoLocs);
				for (GeographicPoint neighborGeoPoint : neighborGeoLocs) {
					MapNode neighborNode = mapGraphVertices.get(neighborGeoPoint);
					Set<MapEdge> edgeSet = currentNode.getEdges();
					MapEdge edge = null;
					double edgeDuration=0;
					Iterator <MapEdge> edgeIterator = edgeSet.iterator();
					while (edgeIterator.hasNext()) {
						edge = new MapEdge();
						edge = edgeIterator.next();
						//System.out.println("neighborNode.getLocation() ="+ neighborNode.getLocation() + " edge.getEndLatLong() ="+edge.getEndLatLong());
							if (neighborNode.getLocation().equals(edge.getEndLatLong())) {
							//System.out.println("Found the edge whose startpoint is the currentNode "+ currentNode.getLocation());
							//System.out.println("Edge start point and end point are "+edge.getStartLatLong() + " "+ edge.getEndLatLong());
							edgeDuration =0;
							//edgeDuration = edge.getLength();
							edgeDuration = getTimeForThisEdge(edge);
							//System.out.println(" Am I infinity neighborNode= "+ neighborNode.getActualDistFrmStart());
							//System.out.println(" Am I infinity currentNode= " + currentNode.getActualDistFrmStart());
								if (Double.POSITIVE_INFINITY!=neighborNode.getActualDurFrmStart()) {
									double tempDistFrmStart = currentNode.getActualDurFrmStart() + edgeDuration;
									//System.out.println( "tempDistFrmStart ="+tempDistFrmStart);
										if (neighborNode.getActualDurFrmStart() > tempDistFrmStart){
											//System.out.println( "tempDistFrmStart ="+tempDistFrmStart);
											//System.out.println("neighborNode.getActualDistFrmStart() > tempDistFrmStart ="+(neighborNode.getActualDistFrmStart() > tempDistFrmStart));
											neighborNode.setActualDurFrmStart(tempDistFrmStart);
											//System.out.println( "Adding "+ currentNode + " as parent of " + neighborNode);
											parentMap.put(neighborNode, currentNode);
										}
								} else {
									double tempDistFrmStart = currentNode.getActualDurFrmStart() + edgeDuration;
					//				System.out.println("actual Dist of neighborNode to be updated since not infinity is "+neighborNode + " is "+ tempDistFrmStart);
									neighborNode.setActualDurFrmStart(tempDistFrmStart);
						//			System.out.println("neighborNode.getActualDistFrmStart() > tempDistFrmStart ="+(neighborNode.getActualDistFrmStart() > tempDistFrmStart));
									neighborNode.setActualDurFrmStart(tempDistFrmStart);
							//		System.out.println( "Adding "+ currentNode + " as parent of " + neighborNode);
									parentMap.put(neighborNode, currentNode);
								}
							}
					}
				
					pQueue.add(neighborNode);
				}
			}
		}		
			if (!currentNode.getLocation().equals(goal)){
				return null;
			}				
				/*System.out.println(" printing parentMap.....");
				for (Entry<MapNode, MapNode> entry : parentMap.entrySet()){
					System.out.println(" entry in parentMap = "+ entry.getKey() +" "+entry.getValue());
				}*/
				System.out.println("going to call a*....");
				return buildAStarRoutePath(parentMap,goal,start);
				//return null;
		} else {
				return null;
			}
	}
	
	
	private List <GeographicPoint> buildAStarRoutePath(Map<MapNode, MapNode>parentMap, GeographicPoint goal, GeographicPoint start){
		//preparing a linked list of geographic locations
		//System.out.println("Did i reach here buildDijkstraRoutePath..");
		List <GeographicPoint> djikstraGeoLocList = new LinkedList <GeographicPoint>();
		//add goal to the route
		djikstraGeoLocList.add(goal);
		boolean startGeoLocFound = false;
		// setting the current location as the destination location to trace back to the source location
		GeographicPoint currentGeoLoc = goal;
		//System.out.println("buildDijkstraRoutePath startGeoLocFound ="+startGeoLocFound);
		//System.out.println(" buildDijkstraRoutePath parentMap size = "+parentMap.entrySet().size());
		//run the loop until the start location is found in the parentMapReceived
		while (startGeoLocFound != true) {
			//System.out.println(" I enter the dragon");
			//iterating through each of the elements of the parentMap
			
			for (Entry<MapNode, MapNode> entry : parentMap.entrySet()) {
				//System.out.println("Enter the map...");
				// the current location is exactly whose parent I am tracing back right upto the start location
				if (currentGeoLoc.equals(entry.getKey().getLocation())) {
					//System.out.println("TRUE entry.getKey() " + entry.getKey() + " currentGeoLoc ="+ currentGeoLoc );
					//setting the parent found as the current location to trace it's parent in next iteration of for loop
					currentGeoLoc = entry.getValue().getLocation();
					//adding the parent found to the route list
					djikstraGeoLocList.add(currentGeoLoc);
				}
				if (currentGeoLoc.equals(start)) {
					// the current location is actually the start location so hurrah break
			//		System.out.println(" currentGeoLoc == start =" + currentGeoLoc + " start =" +start);
					startGeoLocFound = true;
				}
				if(startGeoLocFound) break;
			}
		}
		//System.out.println(djikstraGeoLocList);
		Collections.reverse(djikstraGeoLocList);
		return djikstraGeoLocList;
}

	
	public static void main(String[] args) {
		
			System.out.print("Making a new map...");
			/*MapGraph theMap = new MapGraph();
			System.out.print("DONE. \nLoading the map...");
			GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
			System.out.println("DONE.");*/
			
			MapGraph theMap = new MapGraph();
			System.out.print("DONE. \nLoading the map...");
			GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
			System.out.println("DONE.");

			GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
			GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

			//GeographicPoint start = new GeographicPoint(1.0,1.0);
			//GeographicPoint end = new GeographicPoint(4.0, 4.0);
			List<GeographicPoint> route = theMap.dijkstra(start,end);
			//List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
			System.out.println(" dijkstraCount =" + dijkstraCount);
			System.out.println("dijkstra route = "+route);
			//System.out.println(" aStarCount =" + aStarCount);
			//System.out.println("a* route = "+route2);
			
			/*List<GeographicPoint> route2 = theMap2.aStarSearch(start,end);
			
			
			
			System.out.println("aStarRoute = "+route2);*/
			
			// You can use this method for testing.  
			
			//Use this code in Week 3 End of Week Quiz
/*			MapGraph theMap2 = new MapGraph();
			System.out.print("DONE. \nLoading the map...");
			GraphLoader.loadRoadMap("data/maps/utc.map", theMap2);
			System.out.println("DONE.");

			GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
			GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
			
			
			List<GeographicPoint> route = theMap2.dijkstra(start,end);
			System.out.println("Djikstra route = "+route);
			List<GeographicPoint> route2 = theMap2.aStarSearch(start,end);
			System.out.println("aStarRoute = "+route2);*/

			
		}	
}
