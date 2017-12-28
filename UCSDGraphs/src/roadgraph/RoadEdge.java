package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;
import roadgraph.RoadNode;

public class RoadEdge {
	
	private RoadNode start;
	private RoadNode end;
	private String name;
	private String type;
	private double length;
	
	public RoadEdge(RoadNode from, 
			RoadNode to, 
			String roadName,
			String roadType, 
			double length) 
	{
		this.start = from;
		this.end = to;
		this.name = roadName;
		this.type = roadType;
		this.length = length;
	}
	
	public GeographicPoint getStart()
	{
		return start.getLocation();
	}
	
	public GeographicPoint getEnd()
	{
		return end.getLocation();
	}
	
	public String getName()
	{
		return this.name;
	}
	
	public String getType()
	{
		return this.type;
	}
	
	public double getLength()
	{
		return this.length;
	}
	
	public RoadNode getOtherNode(RoadNode node)
	{
		if (node.equals(start)) {
			return end;
		} else if (node.equals(end)) {
			return start;
		} else {
			throw new IllegalArgumentException("Node does not exist!");
		}
	}
}
