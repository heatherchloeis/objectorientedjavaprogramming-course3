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
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;
import roadgraph.RoadNode;
import roadgraph.RoadEdge;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	//Probably want to utilize an adjacency list 
	private HashMap<GeographicPoint, RoadNode> map;
	private HashSet<RoadEdge> edges;
	Map<GeographicPoint, AStarRoadNode> listAStar = new HashMap<GeographicPoint, AStarRoadNode>();
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		map = new HashMap<GeographicPoint, RoadNode>();
		edges = new HashSet<RoadEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return map.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return map.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		RoadNode node = map.get(location);
		if (node == null) {
			node = new RoadNode(location);
			map.put(location, node);
			return true;
		} else {
			System.out.println("Location already exists!");
		}
		
		return false;
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

		//TODO: Implement this method in WEEK 3
		RoadNode start = map.get(from);
		RoadNode end = map.get(to);
		if (start == null || end == null || length < 0) {
			throw new IllegalArgumentException("Point does not exist!");
		} else {
			RoadEdge edge = new RoadEdge(start, end, roadName, roadType, length);
			edges.add(edge);
			start.addEdge(edge);
		}
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
        return bfs(start, goal, temp);
	}
	
	//Private Helper Method to get neighbor Nodes 
	
	private Set<RoadNode> getNeighbors(RoadNode node) {
		return node.getNeighbors();
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
			 					     GeographicPoint goal, 
			 					     Consumer<GeographicPoint> nodeSearched)
	{
		//Initialize start and end Nodes
		//and check whether or not the nodes are null
		
		RoadNode sNode = map.get(start);
		RoadNode gNode = map.get(goal);
		
		if (sNode == null || gNode == null) {
			System.out.println("Point does not exist! Cannot search for path!");
			return null;
		}
		
		//Initialize Map, Set, and Queue for search
		//and add start Node to the beginning of the Queue
		
		HashMap<RoadNode, RoadNode> parent = new HashMap<RoadNode, RoadNode>();
		HashSet<RoadNode> checked = new HashSet<RoadNode>();
		Queue<RoadNode> toCheck = new LinkedList<RoadNode>();
		
		toCheck.add(sNode);
		RoadNode curr = null;
		
		/**While the Queue is not empty and the current Node does not
		 * equal the goal Node, fetch the neighbor Nodes of the current
		 * and add those to the Map, Set, and Queue 
		 * Continue the loop until the current Node equals the goal Node
		 */
		
		while (!toCheck.isEmpty()) {
			curr = toCheck.remove();
			nodeSearched.accept(curr.getLocation());
			
			if (curr.equals(gNode)) {
				break;
			}
			Set<RoadNode> neighbors = getNeighbors(curr);
			for (RoadNode neighbor : neighbors) {
				if (!checked.contains(neighbor)) {
					checked.add(neighbor);
					parent.put(neighbor, curr);
					toCheck.add(neighbor);
				}
			}
		}
		
		if (!curr.equals(gNode)) {
			System.out.println("Path does not exist!");
			return null;
		}
		
		List<GeographicPoint> path = constructPath(sNode, gNode, parent);
		return path;		
	}
	
	private List<GeographicPoint> constructPath(RoadNode start, RoadNode goal, HashMap<RoadNode, RoadNode> parent) 
	{
		/**Private Helper method to construct the path
		 * While the current Node does not equal the Goal Node
		 * The loop will continue to add Nodes to the front of the List,
		 * Retrieve a new Node from the parent Map, and repeat the loop
		 * The List is finished when the current Node equals the Goal Node
		 */
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		RoadNode curr = goal;
		while (!curr.equals(start)) {
			path.addFirst(curr.getLocation());
			curr = parent.get(curr);
		}
		path.addFirst(start.getLocation());
		return path;
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
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, 
										  Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//Initialize start and end Nodes
		//and check whether or not the nodes are null
		
		RoadNode sNode = map.get(start);
		RoadNode gNode = map.get(goal);
		
		if (sNode == null || gNode == null) {
			System.out.println("Point does not exist! Cannot search for path!");
			return null;
		}
		
		//Initialize Map, Set, and Queue for search
		//and add start Node to the beginning of the Queue
		
		HashMap<RoadNode, RoadNode> parent = new HashMap<RoadNode, RoadNode>();
		HashSet<RoadNode> checked = new HashSet<RoadNode>();
		PriorityQueue<RoadNode> toCheck = new PriorityQueue<RoadNode>();
		//Integer for Lecture Quiz only!!!
		int count = 0;
		
		//Initialize all distances of all possible Nodes to infinity
		
		for (RoadNode node : map.values()) {
			node.setDistance(Double.POSITIVE_INFINITY);
		}
		
		//Set start Node distance from start Node to 0.0
		
		sNode.setDistance(0.0);
		
		toCheck.add(sNode);
		RoadNode curr = null;
		
		/**While the Queue is not empty and the Set does not
		 * already contain the curr Node, add curr Node to Set
		 */
		
		while (!toCheck.isEmpty()) {
			curr = toCheck.remove();
			nodeSearched.accept(curr.getLocation());
			count++;
			
			if (!checked.contains(curr)) {
				checked.add(curr);
				if (curr.equals(gNode)) {
					break;
				}
				
				/**Because we are searching by least total edges and not by
				 * least number of Nodes visited, we will traverse the mapgraph
				 * by edges and not by vertices
				 */
				
				Set<RoadEdge> edges = curr.getEdges();
				for (RoadEdge edge : edges) {
					
					//Get neighbor Nodes from edges of curr Node
					
					RoadNode neighbor = edge.getOtherNode(curr);
					if (!checked.contains(neighbor)) {
						
						/**Update current Distance from start Node to curr Node.
						 * If current Distance is less than the distance from
						 * start Node to neighbor Node, add neighbor to Queue.
						 * Update neighbor Node distance to current Distance.
						 * Add neighbor Node to Map
						 */
						
						double currDistance = curr.getDistance() + edge.getLength();
						if (currDistance < neighbor.getDistance()) {
							neighbor.setDistance(currDistance);
							parent.put(neighbor, curr);
							toCheck.add(neighbor);
						}
					}
				}
			}
		}
		
		if (!curr.equals(gNode)) {
			System.out.println("Path does not exist!");
			return null;
		}
		
		System.out.println("For Dijkstra, RoadNodes visited: " + count);
		
		List<GeographicPoint> path = constructPath(sNode, gNode, parent);
		return path;
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
	
		public List<GeographicPoint> greedySearch(GeographicPoint start, 
											List<GeographicPoint> list)
	{
		if (list.contains(start)) {
			list.remove(start);
		}
		List<GeographicPoint> greedyList = new ArrayList<GeographicPoint>();
		GeographicPoint curr = start;
		
		while (!list.isEmpty()) {
			GeographicPoint next = list.get(0);
			GeographicPoint first = next;
			aStarSearch(curr, next);
			double minDist = map.get(next).getDistance();
			
			for (int i = 0; i < list.size(); i++) {
				next = list.get(i);
				aStarSearch(curr, next);
				double dist = listAStar.get(next).getDistance();
				
				if (dist < minDist) {
					first = next;
					minDist = dist;
				}
			}
		}
		
		return greedyList;
	}
	
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
				 							GeographicPoint goal, 
				 							Consumer<GeographicPoint> nodeSearched)
	{		
		// Initialize data structures
		Queue<AStarRoadNode> queue = new PriorityQueue<AStarRoadNode>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap
		<GeographicPoint, GeographicPoint>();
		
		
		
		// Copy all vertexList objects to vertexListAStar
		for(GeographicPoint geoPoint: map.keySet()){
			listAStar.put(geoPoint, new AStarRoadNode(geoPoint, 
					Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
		}
		
		// Make all CurrentDistances of the MapNodeAStar objects to infinity
		for(AStarRoadNode roadNode: listAStar.values()){
			roadNode.setDistance(Double.POSITIVE_INFINITY);
		}
		
		// Implementation of the A-Star algorithm
		queue.add(new AStarRoadNode(start, 0, 0));
		
		while(!queue.isEmpty()){
			AStarRoadNode curr = queue.remove();
		
		
		// Hook for visualization.
			nodeSearched.accept(curr.getLocation());
		
			if(!visited.contains(curr.getLocation())){
					visited.add(curr.getLocation());
				if(curr.getLocation().equals(goal)){
					List<GeographicPoint> shortestPath = new ArrayList<GeographicPoint>();
					shortestPath.add(curr.getLocation());
				
					GeographicPoint child = parentMap.get(curr.getLocation());
				
					while(!child.equals(start)){
						shortestPath.add(child);
						child = parentMap.get(child);
					}
					shortestPath.add(start);
					Collections.reverse(shortestPath);
					
					return shortestPath;
				}
				
				for(RoadEdge n : curr.getEdges()){
					if (!visited.contains(n.getEnd())){			
						if(curr.getActualDistance() + n.getLength() < 
						listAStar.get(n.getEnd()).getActualDistance()) {
						
								listAStar.get(n.getEnd()).setDistance(n.getLength() 
									+ curr.getActualDistance());
								listAStar.get(n.getEnd()).setPredictedDistance(goal);
								parentMap.put(listAStar.get(n.getEnd()).getLocation(), curr.getLocation());
								queue.add(listAStar.get(n.getEnd()));
						}
					}
				}
			}
		}
		// The path from start to goal does not exist		
	return null;
	}

	
	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
//		MapGraph simpleTestMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//		
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//		
//		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
//		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
//		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
		MapGraph simpleTestMap = new MapGraph();
			GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
	}
	
}
