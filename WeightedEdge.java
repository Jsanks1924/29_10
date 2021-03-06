//From book

public class WeightedEdge extends AbstractGraph.Edge
        implements Comparable<WeightedEdge> {
    public double weight; // The weight on edge (u, v)

    public WeightedEdge(int u, int v, double weight) {
        super(u, v);
        this.weight = weight;
    }

    public int compareTo(WeightedEdge edge) {
        if (weight > edge.weight) {
            return 1;
        }
        else if (weight == edge.weight) {
            return 0;
        }
        else {
            return -1;
        }
    }
}