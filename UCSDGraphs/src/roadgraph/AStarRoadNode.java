package roadgraph;

import geography.GeographicPoint;

public class AStarRoadNode extends RoadNode 
{
	private double predictedDistance;
	
	public AStarRoadNode(GeographicPoint pt) 
	{
		this.setLocation(pt);
	}
	
	public AStarRoadNode(GeographicPoint pt, double currDist, double predDist)
	{
		this.setLocation(pt);
		this.setDistance(currDist);
		this.predictedDistance = predDist;
	}
	
	@Override
	public int compareTo(RoadNode node)
	{
		return Double.compare(this.getActualDistance(), ((AStarRoadNode)node).getActualDistance());
	}
	
	public double getActualDistance() {
		return predictedDistance;
	}
	
	public void setPredictedDistance(GeographicPoint start)
	{
		this.predictedDistance = this.getDistance() + this.getLocation().distance(start);
	}
}
