/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import java.lang.reflect.Array;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 *         <p>
 *         A class which represents a graph of geographic locations
 *         Nodes in the graph are intersections between
 */
public class MapGraph {
    //TODO: Add your member variables here in WEEK 2
    private Map<GeographicPoint, MapVertex> mapGraphVertices;
    private static AtomicInteger dijkstraCounter;
    private static AtomicInteger aStarCounter;

    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        // TODO: Implement in this constructor in WEEK 2
        this.mapGraphVertices = new HashMap<>();
        dijkstraCounter = new AtomicInteger();
        aStarCounter = new AtomicInteger();
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        //TODO: Implement this method in WEEK 2
        return this.mapGraphVertices.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        //TODO: Implement this method in WEEK 2
        return new HashSet<>(this.mapGraphVertices.keySet());
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        //TODO: Implement this method in WEEK 2
        int edgeCounter = 0;
        for (GeographicPoint key : getVertices()) {
            edgeCounter += this.mapGraphVertices.get(key).getNeighbours().size();
        }
        return edgeCounter;
    }


    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        // TODO: Implement this method in WEEK 2
        if (!(location == null) && !(this.mapGraphVertices.containsKey(location))) {
            this.mapGraphVertices.put(location, new MapVertex(location));
            return true;
        }

        return false;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {

        //TODO: Implement this method in WEEK 2
        if (from == null || !this.mapGraphVertices.containsKey(from) || to == null
                || !this.mapGraphVertices.containsKey(to) || roadName == null
                || roadType == null || length < 0) {
            throw new IllegalArgumentException("The edge arguement are not correct.. Please check the start and end point are already intersection" +
                    " in the grapth, and make sure the road length is not less than Zero");
        }


        // create an edge object && reuse it after swapping the endpoints. Just to save some memory.
        MapEdge dummyEdge = new MapEdge(from, to, roadName, roadType, length);
        // adding the edge to the "FROM" neighbour list
        MapVertex fromVertex = this.mapGraphVertices.get(from);
        fromVertex.getNeighbours().add(dummyEdge);

    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 2
        // initialization of the used DataStructure,
        LinkedList<MapVertex> queue = new LinkedList<>();
        HashSet<MapVertex> visitedVertices = new HashSet<>();
        HashMap<MapVertex, ArrayList<MapVertex>> parentMap = new HashMap<>();

        // Casting for the start and goal vertices.
        MapVertex startVertex = this.mapGraphVertices.get(start);
        MapVertex goalVertex = this.mapGraphVertices.get(goal);

        // starting the search.
        queue.addLast(startVertex);
        visitedVertices.add(startVertex);
        // checking the result of the search, and returning the path ifExist.
        boolean found = bfs(queue, visitedVertices, parentMap, goalVertex);
        if (found) {
            return buildPathFromParentMap(parentMap, startVertex, goalVertex);
        }

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        // if no path was found, return NULL.
        return null;
    }

    // added by Mustafa

    /**
     * Find if there is a path to the goal vertex in the graph.
     *
     * @param queue           A queue for processing vertices in a level fashion.
     * @param visitedVertices A HashSet for checking if a vertex was visited before.
     * @param parentMap       A HashMap contains the path from start till the destination.
     * @param goal            A destination vertex.
     * @return A boolean indicate the result pf the search.
     */

    private boolean bfs(LinkedList<MapVertex> queue, HashSet<MapVertex> visitedVertices, HashMap<MapVertex, ArrayList<MapVertex>> parentMap, MapVertex goal) {

        while (!queue.isEmpty()) {

            // get the first Vertex in the queue.
            MapVertex current = queue.removeFirst();
            // check if the goal has reached
            if (current.equals(goal)) {
                return true;
            }

            // processing current's neighbours.
            for (MapEdge edge : current.getNeighbours()) {

                // retrieve the Vertex from the Graph.
                MapVertex n = this.mapGraphVertices.get(edge.getEnd());
                // make sure it was not visited before.
                if (!visitedVertices.contains(n)) {
                    visitedVertices.add(n);
                    if (parentMap.containsKey(current)) {
                        parentMap.get(current).add(n);
                    } else {
                        ArrayList<MapVertex> list = new ArrayList<>();
                        list.add(n);
                        parentMap.put(current, list);
                    }
                    queue.addLast(n);
                }
            }
        }

        return false;
    }


    /**
     * A helper method to build the path of the search, from start to goal, using the parent HashMap.
     *
     * @param parentMap A map contains each vertex as Value, and its parent in the graph as a Key.
     * @param start     A start vertex in the Graph.
     * @param goal      A goal Vertex in the Graph.
     * @return A list contains the vertices in the path from start to end.
     */
    private List<GeographicPoint> buildPathFromParentMap(HashMap<MapVertex, ArrayList<MapVertex>> parentMap, MapVertex start, MapVertex goal) {

        // filling the list with the inverse path, from end to start
        ArrayList<GeographicPoint> shortestPath = new ArrayList<>();
        Set<Map.Entry<MapVertex, ArrayList<MapVertex>>> parentSet = parentMap.entrySet();
        MapVertex current = goal;
        shortestPath.add(goal.getGeographicPoint());

        while (!current.equals(start)) {
            for (Map.Entry<MapVertex, ArrayList<MapVertex>> entry : parentSet) {
                if (entry.getValue().contains(current)) {
                    shortestPath.add(entry.getKey().getGeographicPoint());
                    current = entry.getKey();
                    break;
                }
            }
        }


        // reverse the order to get from Start to End
        Collections.reverse(shortestPath);

        return shortestPath;
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3
        PriorityQueue<MapVertex> queue = new PriorityQueue<>();
        HashSet<MapVertex> visitedVertices = new HashSet<>();
        HashMap<MapVertex, MapVertex> parentMap = new HashMap<>();
        HashMap<MapVertex, Double> distances = new HashMap<>();

        // Casting for the start and goal vertices.
        MapVertex startVertex = this.mapGraphVertices.get(start);
        MapVertex goalVertex = this.mapGraphVertices.get(goal);

        // starting the search.
        startVertex.setPriority(0);
        distances.put(startVertex, 0.0);
        queue.offer(startVertex);
        visitedVertices.add(startVertex);

        // checking the result of the search, and returning the path ifExist.
        boolean found = bfs(queue, visitedVertices, parentMap, distances, goalVertex, false, dijkstraCounter);
        if (found) {
            System.out.println(" DIJKSTRA  ******************************** " + dijkstraCounter);
            return buildPathFromDijkstraParentMap(parentMap, startVertex, goalVertex);
        }


        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }


    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3
        PriorityQueue<MapVertex> queue = new PriorityQueue<>(100, new MapVertexHeuristic());
        HashSet<MapVertex> visitedVertices = new HashSet<>();
        HashMap<MapVertex, MapVertex> parentMap = new HashMap<>();
        HashMap<MapVertex, Double> distances = new HashMap<>();

        // Casting for the start and goal vertices.
        MapVertex startVertex = this.mapGraphVertices.get(start);
        MapVertex goalVertex = this.mapGraphVertices.get(goal);

        // starting the search.
        startVertex.setPriority(0);
        startVertex.setPredictedDistance(0);
        distances.put(startVertex, 0.0);
        queue.offer(startVertex);
        visitedVertices.add(startVertex);

        // checking the result of the search, and returning the path ifExist.
        boolean found = bfs(queue, visitedVertices, parentMap, distances, goalVertex, true, aStarCounter);
        if (found) {
            System.out.println(" ASTAR  ******************************** " + aStarCounter);
            return buildPathFromDijkstraParentMap(parentMap, startVertex, goalVertex);
        }

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }

    /**
     * @param queue           A queue for processing vertices in a level fashion.
     * @param visitedVertices A HashSet for checking if a vertex was visited before.
     * @param parentMap       A HashMap contains the path from start till the destination.
     * @param goal            A destination vertex.
     * @param distances       A Map contains each vertex, and its shortest path.
     * @return A boolean indicate the result pf the search.
     */
    private boolean bfs(PriorityQueue<MapVertex> queue, HashSet<MapVertex> visitedVertices, HashMap<MapVertex, MapVertex> parentMap, HashMap<MapVertex, Double> distances, MapVertex goal, boolean aStar, AtomicInteger visitedCounter) {



        while (!queue.isEmpty()) {

            // get the first Vertex in the queue.
            MapVertex current = queue.poll();
            // counter for removed vertices
            visitedCounter.getAndIncrement();

            if (!visitedVertices.contains(current)) {
                visitedVertices.add(current);
            }
            // check if the goal has reached
            if (current.equals(goal)) {
                return true;
            }

            // processing current's neighbours.
            for (MapEdge edge : current.getNeighbours()) {

                // retrieve the Vertex from the Graph.
                MapVertex n = this.mapGraphVertices.get(edge.getEnd());
                // make sure it was not visited before.
                if (!visitedVertices.contains(n)) {

                    // sanity Check
                    if (!distances.containsKey(n)) {
                        distances.put(n, Double.MAX_VALUE);
                    }

                    // dijkstra distance updates
                    double nPriority = current.getPriority() + edge.getLength();

                    if (nPriority < distances.get(n)) {
                        distances.put(n, nPriority);
                        n.setPriority(nPriority);
                        n.setParentEdgeLength(edge.getLength());

                        // A-STAR Logic
                        if (aStar) {
                            double straightLineDistance = n.getGeographicPoint().distance(goal.getGeographicPoint());
                            n.setPredictedDistance(n.getPriority() + straightLineDistance);
                        }
                        // update Parent
                        parentMap.put(n, current);

                        // enqueue
                        queue.offer(n);
                    }
                }
            }
        }

        return false;
    }

    /**
     * A helper method to build the path of the search, from start to goal, using the parent HashMap.
     *
     * @param parentMap A map contains each vertex as Key, and its parent in the graph as a Value.
     * @param start     A start vertex in the Graph.
     * @param goal      A goal Vertex in the Graph.
     * @return A list contains the vertices in the path from start to end.
     */
    private List<GeographicPoint> buildPathFromDijkstraParentMap(HashMap<MapVertex, MapVertex> parentMap, MapVertex start, MapVertex goal) {

        // filling the list with the inverse path, from end to start
        ArrayList<GeographicPoint> shortestPath = new ArrayList<>();
        //Set<Map.Entry<MapVertex, ArrayList<MapVertex>>> parentSet = parentMap.entrySet();
        MapVertex current = goal;
        shortestPath.add(goal.getGeographicPoint());
        while (!current.equals(start)) {
            if (parentMap.containsKey(current)) {
                shortestPath.add(parentMap.get(current).getGeographicPoint());
                current = parentMap.get(current);
            }
        }
        // reverse the order to get from Start to End
        Collections.reverse(shortestPath);

        return shortestPath;
    }


    public static void main(String[] args) {

        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

        List<GeographicPoint> route = theMap.dijkstra(start,end);
        List<GeographicPoint> route2 = theMap.aStarSearch(start,end);


       /* System.out.print("Making a new map...");
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        System.out.println("DONE.");

        // You can use this method for testing.

		*//* Use this code in Week 3 End of Week Quiz
        MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*//*
*/
    }

}
