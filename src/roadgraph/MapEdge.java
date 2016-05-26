package roadgraph;

import geography.GeographicPoint;

/**
 * Created by mustafa on 1/29/16.
 */
public class MapEdge {

    private GeographicPoint start;
    private GeographicPoint end;
    private String roadName;
    private String roadType;
    private double length;

    public MapEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {

        this.start = from;
        this.end = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public GeographicPoint getStart() {
        return start;
    }

    public void setStart(GeographicPoint start) {
        this.start = start;
    }

    public GeographicPoint getEnd() {
        return end;
    }

    public void setEnd(GeographicPoint end) {
        this.end = end;
    }

    public String getRoadName() {
        return roadName;
    }

    public void setRoadName(String roadName) {
        this.roadName = roadName;
    }

    public String getRoadType() {
        return roadType;
    }

    public void setRoadType(String roadType) {
        this.roadType = roadType;
    }

    public double getLength() {
        return length;
    }

    public void setLength(double length) {
        this.length = length;
    }
}
