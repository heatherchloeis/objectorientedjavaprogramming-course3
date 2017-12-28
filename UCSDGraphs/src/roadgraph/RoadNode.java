package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

public class RoadNode implements Comparable<RoadNode> {
	
	private GeographicPoint location;
	private HashSet<RoadEdge> edges;
	private double distance;
	private double actualDistance;
	
	public RoadNode() {}
	
	public RoadNode(GeographicPoint point)
	{
		location = point;
		edges = new HashSet<RoadEdge>();
	}
	
	public void setLocation(GeographicPoint pt)
	{
		this.location = pt;
	}
	
	public GeographicPoint getLocation()
	{
		return location;
	}
	
	public void addEdge(RoadEdge edge)
	{
		edges.add(edge);
	}
	
	public Set<RoadEdge> getEdges() {
		return edges;
	}
	
	RoadEdge getEdge( RoadNode node) {
		for ( RoadEdge e : edges) {
			if (e.getOtherNode(this) == node) return e;
		}
		return null;
	}
	
	public Set<RoadNode> getNeighbors() 
	{
		Set<RoadNode> neighbors = new HashSet<RoadNode>();
		for (RoadEdge re : edges) {
			neighbors.add(re.getOtherNode(this));
		}
		return neighbors;
	}
	
	public double getLength(RoadNode other) {
		for (RoadEdge edge : edges) {
			if (this.getLocation().equals(edge.getStart()) && other.getLocation().equals(edge.getEnd())) {
				return edge.getLength();
			}
		}
		return 0.0;
	}
	
	public double getDistance() {
		return this.distance;
	}
	
	// set node distance (predicted)
	public void setDistance(double distance) {
	    this.distance = distance;
	}

	// get node distance (actual)
	public double getActualDistance() {
		return this.actualDistance;
	}
	
	// set node distance (actual)	
	public void setActualDistance(double actualDistance) {
	    this.actualDistance = actualDistance;
	}
	
	@Override public int compareTo(RoadNode other) 
	{
		return Double.compare(this.getDistance(), other.getDistance());
	}
}
