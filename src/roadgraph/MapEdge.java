package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	
	private GeographicPoint startLatLong;
	
	private GeographicPoint endLatLong;
	
	private String roadName = "";
	
	private String roadType ="";
	
	private double length = 0;

	//default constructor
	public MapEdge() {
		startLatLong = new GeographicPoint(0, 0);
		endLatLong = new GeographicPoint(0, 0);
		roadName="";
		roadType="";
		length=0;
	}

	/**
	 * @return the startLatLong
	 */
	public GeographicPoint getStartLatLong() {
		return startLatLong;
	}

	/**
	 * @param startLatLong the startLatLong to set
	 */
	public void setStartLatLong(GeographicPoint startLatLong) {
		this.startLatLong = startLatLong;
	}

	/**
	 * @return the endLatLong
	 */
	public GeographicPoint getEndLatLong() {
		return endLatLong;
	}

	/**
	 * @param endLatLong the endLatLong to set
	 */
	public void setEndLatLong(GeographicPoint endLatLong) {
		this.endLatLong = endLatLong;
	}

	/**
	 * @return the roadName
	 */
	public String getRoadName() {
		return roadName;
	}

	/**
	 * @param roadName the roadName to set
	 */
	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}

	/**
	 * @return the roadType
	 */
	public String getRoadType() {
		return roadType;
	}

	/**
	 * @param roadType the roadType to set
	 */
	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}

	/**
	 * @return the length
	 */
	public double getLength() {
		return length;
	}

	/**
	 * @param length the length to set
	 */
	public void setLength(double length) {
		this.length = length;
	}
	
}
