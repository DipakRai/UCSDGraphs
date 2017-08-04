package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;
/**
 * 
 * @author dipakrai
 * The class represents a node/intersection on the map
 * The class has two attributes location and edgeSet
 * location represents the latitude, longitude coordinates of the node/intersection and
 * edgeSet represents the paths/roads/streets from/to the node/road/intersections
 * 
 */
public class MapNode implements Comparable<MapNode>{
	
	private GeographicPoint location;
	
	private Set<MapEdge> edges;
	
	private double actualDurFrmEnd = Double.POSITIVE_INFINITY;
	
	private double actualDurFrmStart = Double.POSITIVE_INFINITY;
	
	public MapNode() {
		location = new GeographicPoint(0,0);
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * @return the location
	 */
	public GeographicPoint getLocation() {
		return location;
	}

	/**
	 * @param location the location to set
	 */
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	/**
	 * @return the edges
	 */
	public Set<MapEdge> getEdges() {
		return edges;
	}

	/**
	 * @param edges the edges to set
	 */
	public void setEdges(Set<MapEdge> edges) {
		this.edges = edges;
	}


	public double getActualDurFrmEnd() {
		return actualDurFrmEnd;
	}

	public void setActualDurFrmEnd(double actualDurFrmEnd) {
		this.actualDurFrmEnd = actualDurFrmEnd;
	}

	public double getActualDurFrmStart() {
		return actualDurFrmStart;
	}

	public void setActualDurFrmStart(double actualDurFrmStart) {
		this.actualDurFrmStart = actualDurFrmStart;
	}

	@Override
	public int compareTo(MapNode node) {
		//return ((Double)(this.actualDurFrmStart + this.actualDurFrmEnd)).compareTo((Double)(node.getActualDurFrmStart() + node.getActualDurFrmEnd()));
		return ((Double)(this.actualDurFrmStart)).compareTo((Double)(node.getActualDurFrmStart()));
	}

	@Override
	public String toString() {
		return "MapNode [location=" + location + ", edges=" + edges + ", actualDurFrmEnd=" + actualDurFrmEnd
				+ ", actualDurFrmStart=" + actualDurFrmStart + "]";
	}
	
}