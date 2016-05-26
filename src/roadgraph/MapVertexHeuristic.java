package roadgraph;

import java.util.Comparator;

/**
 * Created by mustafa on 1/30/16.
 */
public class MapVertexHeuristic implements Comparator<MapVertex> {

    @Override
    public int compare(MapVertex o1, MapVertex o2) {
        if (o1.getPredictedDistance() < o2.getPredictedDistance()) {
            return -1;
        } else if (o1.getPredictedDistance() > o2.getPredictedDistance()) {
            return 1;
        } else
            return 0;
    }
}
