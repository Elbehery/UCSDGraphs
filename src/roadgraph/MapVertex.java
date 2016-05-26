package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;


public class MapVertex implements Comparable<MapVertex> {

    private GeographicPoint geographicPoint;
    private List<MapEdge> neighbours;
    private double priority;
    private double parentEdgeLength;
    private double predictedDistance;

    public MapVertex(GeographicPoint geographicPoint) {
        this.geographicPoint = geographicPoint;
        this.neighbours = new ArrayList<>();
        this.priority = Double.MAX_VALUE;
        this.predictedDistance = Double.MAX_VALUE;
    }

    public GeographicPoint getGeographicPoint() {
        return geographicPoint;
    }

    public void setGeographicPoint(GeographicPoint geographicPoint) {
        this.geographicPoint = geographicPoint;
    }

    public List<MapEdge> getNeighbours() {
        return neighbours;
    }

    public void setNeighbours(List<MapEdge> neighbours) {
        this.neighbours = neighbours;
    }

    public double getPriority() {
        return priority;
    }

    public void setPriority(double priority) {
        this.priority = priority;
    }

    public double getParentEdgeLength() {
        return parentEdgeLength;
    }

    public void setParentEdgeLength(double parentEdgeLength) {
        this.parentEdgeLength = parentEdgeLength;
    }

    public double getPredictedDistance() {
        return predictedDistance;
    }

    public void setPredictedDistance(double predictedDistance) {
        this.predictedDistance = predictedDistance;
    }

    @Override
    public int compareTo(MapVertex o) {
        if (this.priority < o.priority)
            return -1;
        else if (this.priority > o.priority)
            return 1;
        else
            return 0;
    }

    @Override
    public boolean equals(Object obj) {

        MapVertex that = (MapVertex) obj;
        Double thisX = this.getGeographicPoint().getX();
        Double thatX = that.getGeographicPoint().getX();

        Double thisY = this.getGeographicPoint().getY();
        Double thatY = that.getGeographicPoint().getY();

        if (thisX.equals(thatX) && thisY.equals(thatY)) {
            return true;
        }

        return false;
    }

/*    *//**
     * Generate string representation of adjacency list
     *
     * @return the String
     *//*
    public String adjacencyString() {
        String s = "Adjacency list";
        s += " (size " + getNumVertices() + "+" + getNumEdges() + " integers):";

        for (int v : adjListsMap.keySet()) {
            s += "\n\t" + v + ": ";
            for (int w : adjListsMap.get(v)) {
                s += w + ", ";
            }
        }
        return s;
    }*/

}
