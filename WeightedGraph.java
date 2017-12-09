//From book
import java.util.*;

public class WeightedGraph<V> extends AbstractGraph<V> {
    // Priority adjacency lists
    private List<PriorityQueue<WeightedEdge>> queues
            = new ArrayList<PriorityQueue<WeightedEdge>>();

    public WeightedGraph() {
    }

    public WeightedGraph(int[][] edges, V[] vertices) {
        super(edges, vertices);
        createQueues(edges, vertices.length);
    }

    public WeightedGraph(int[][] edges, int numberOfVertices) {
        super(edges, numberOfVertices);
        createQueues(edges, numberOfVertices);
    }

    public WeightedGraph(List<WeightedEdge> edges, List<V> vertices) {
        super((List)edges, vertices);
        createQueues(edges, vertices.size());
    }

    public WeightedGraph(List<WeightedEdge> edges,
                         int numberOfVertices) {
        super((List)edges, numberOfVertices);
        createQueues(edges, numberOfVertices);
    }

    private void createQueues(int[][] edges, int numberOfVertices) {
        for (int i = 0; i < numberOfVertices; i++) {
            queues.add(new PriorityQueue<WeightedEdge>());
        }

        for (int i = 0; i < edges.length; i++) {
            int u = edges[i][0];
            int v = edges[i][1];
            int weight = edges[i][2];
            // Insert an edge into the queue
            queues.get(u).offer(new WeightedEdge(u, v, weight));
        }
    }

    private void createQueues(List<WeightedEdge> edges,
                              int numberOfVertices) {
        for (int i = 0; i < numberOfVertices; i++) {
            queues.add(new PriorityQueue<WeightedEdge>());
        }

        for (WeightedEdge edge: edges) {
            queues.get(edge.u).offer(edge);
        }
    }

    public void printWeightedEdges() {
        for (int i = 0; i < queues.size(); i++) {
            System.out.print(getVertex(i) + " (" + i + "): ");
            for (WeightedEdge edge : queues.get(i)) {
                System.out.print("(" + edge.u +
                        ", " + edge.v + ", " + edge.weight + ") ");
            }
            System.out.println();
        }
    }

    public List<PriorityQueue<WeightedEdge>> getWeightedEdges() {
        return queues;
    }

    public void clear() {
        vertices.clear();
        neighbors.clear();
        queues.clear();
    }

    public void addVertex(V vertex) {
        super.addVertex(vertex);
        queues.add(new PriorityQueue<WeightedEdge>());
    }

    public void addEdge(int u, int v, double weight) {
        super.addEdge(u, v);
        queues.get(u).add(new WeightedEdge(u, v, weight));
        queues.get(v).add(new WeightedEdge(v, u, weight));
    }

    public MST getMinimumSpanningTree() {
        return getMinimumSpanningTree(0);
    }

    public MST getMinimumSpanningTree(int startingVertex) {
        List<Integer> T = new ArrayList<Integer>();
        // T initially contains the startingVertex;
        T.add(startingVertex);

        int numberOfVertices = vertices.size();
        int[] parent = new int[numberOfVertices];
        for (int i = 0; i < parent.length; i++)
            parent[i] = -1;
        double totalWeight = 0;


        List<PriorityQueue<WeightedEdge>> queues = deepClone(this.queues);

        while (T.size() < numberOfVertices) {
            int v = -1;
            double smallestWeight = Double.MAX_VALUE;
            for (int u : T) {
                while (!queues.get(u).isEmpty() &&
                        T.contains(queues.get(u).peek().v)) {
                    queues.get(u).remove();
                }

                if (queues.get(u).isEmpty()) {
                    continue;
                }

                WeightedEdge edge = queues.get(u).peek();
                if (edge.weight < smallestWeight) {
                    v = edge.v;
                    smallestWeight = edge.weight;
                    parent[v] = u;
                }
            }

            if (v != -1)
                T.add(v);
            else
                break;

            totalWeight += smallestWeight;
        }

        return new MST(startingVertex, parent, T, totalWeight);
    }

    private List<PriorityQueue<WeightedEdge>> deepClone(
            List<PriorityQueue<WeightedEdge>> queues) {
        List<PriorityQueue<WeightedEdge>> copiedQueues =
                new ArrayList<PriorityQueue<WeightedEdge>>();

        for (int i = 0; i < queues.size(); i++) {
            copiedQueues.add(new PriorityQueue<WeightedEdge>());
            for (WeightedEdge e : queues.get(i)) {
                copiedQueues.get(i).add(e);
            }
        }

        return copiedQueues;
    }

    public class MST extends Tree {
        private double totalWeight;

        public MST(int root, int[] parent, List<Integer> searchOrder,
                   double totalWeight) {
            super(root, parent, searchOrder);
            this.totalWeight = totalWeight;
        }

        public double getTotalWeight() {
            return totalWeight;
        }
    }

    public ShortestPathTree getShortestPath(int sourceVertex) {
        List<Integer> T = new ArrayList<Integer>();
        T.add(sourceVertex);

        int numberOfVertices = vertices.size();

        int[] parent = new int[numberOfVertices];
        parent[sourceVertex] = -1;

        double[] cost = new double[numberOfVertices];
        for (int i = 0; i < cost.length; i++) {
            cost[i] = Double.MAX_VALUE;
        }
        cost[sourceVertex] = 0;

        List<PriorityQueue<WeightedEdge>> queues = deepClone(this.queues);

        while (T.size() < numberOfVertices) {
            int v = -1;
            double smallestCost = Double.MAX_VALUE;
            for (int u : T) {
                while (!queues.get(u).isEmpty() &&
                        T.contains(queues.get(u).peek().v)) {
                    queues.get(u).remove();
                }

                if (queues.get(u).isEmpty()) {

                    continue;
                }

                WeightedEdge e = queues.get(u).peek();
                if (cost[u] + e.weight < smallestCost) {
                    v = e.v;
                    smallestCost = cost[u] + e.weight;

                    parent[v] = u;
                }
            }

            T.add(v);
            cost[v] = smallestCost;
        }


        return new ShortestPathTree(sourceVertex, parent, T, cost);
    }

    public class ShortestPathTree extends Tree {
        private double[] cost;

        public ShortestPathTree(int source, int[] parent,
                                List<Integer> searchOrder, double[] cost) {
            super(source, parent, searchOrder);
            this.cost = cost;
        }

        public double getCost(int v) {
            return cost[v];
        }

        public void printAllPaths() {
            System.out.println("All shortest paths from " +
                    vertices.get(getRoot()) + " are:");
            for (int i = 0; i < cost.length; i++) {
                printPath(i);
                System.out.println("(cost: " + cost[i] + ")");
            }
        }
    }
}