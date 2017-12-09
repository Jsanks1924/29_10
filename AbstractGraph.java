//modified From book

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;

public abstract class AbstractGraph<V> implements Graph<V> {
    protected List<V> vertices = new ArrayList<>();
    protected List<List<Integer>> neighbors
            = new ArrayList<List<Integer>>();

    protected AbstractGraph() {
    }

    protected AbstractGraph(int[][] edges, V[] vertices) {
        for (int i = 0; i < vertices.length; i++)
            this.vertices.add(vertices[i]);

        createAdjacencyLists(edges, vertices.length);
    }

    protected AbstractGraph(List<Edge> edges, List<V> vertices) {
        for (int i = 0; i < vertices.size(); i++)
            this.vertices.add(vertices.get(i));

        createAdjacencyLists(edges, vertices.size());
    }

    protected AbstractGraph(List<Edge> edges, int numberOfVertices) {
        for (int i = 0; i < numberOfVertices; i++)
            vertices.add((V)(new Integer(i)));

        createAdjacencyLists(edges, numberOfVertices);
    }

    protected AbstractGraph(int[][] edges, int numberOfVertices) {
        for (int i = 0; i < numberOfVertices; i++)
            vertices.add((V)(new Integer(i)));

        createAdjacencyLists(edges, numberOfVertices);
    }

    private void createAdjacencyLists(
            int[][] edges, int numberOfVertices) {
        // Create a linked list
        for (int i = 0; i < numberOfVertices; i++) {
            neighbors.add(new ArrayList<Integer>());
        }

        for (int i = 0; i < edges.length; i++) {
            int u = edges[i][0];
            int v = edges[i][1];
            neighbors.get(u).add(v);
        }
    }

    private void createAdjacencyLists(
            List<Edge> edges, int numberOfVertices) {
        for (int i = 0; i < numberOfVertices; i++) {
            neighbors.add(new ArrayList<Integer>());
        }

        for (Edge edge: edges) {
            neighbors.get(edge.u).add(edge.v);
        }
    }

    @Override
    public int getSize() {
        return vertices.size();
    }

    @Override
    public List<V> getVertices() {
        return vertices;
    }

    @Override
    public V getVertex(int index) {
        return vertices.get(index);
    }

    @Override
    public int getIndex(V v) {
        return vertices.indexOf(v);
    }

    @Override
    public List<Integer> getNeighbors(int index) {
        return neighbors.get(index);
    }

    @Override
    public int getDegree(int v) {
        return neighbors.get(v).size();
    }

    @Override
    public void printEdges() {
        for (int u = 0; u < neighbors.size(); u++) {
            System.out.print(getVertex(u) + " (" + u + "): ");
            for (int j = 0; j < neighbors.get(u).size(); j++) {
                System.out.print("(" + u + ", " +
                        neighbors.get(u).get(j) + ") ");
            }
            System.out.println();
        }
    }

    @Override
    public void clear() {
        vertices.clear();
        neighbors.clear();
    }

    @Override
    public void addVertex(V vertex) {
        vertices.add(vertex);
        neighbors.add(new ArrayList<Integer>());
    }

    @Override
    public void addEdge(int u, int v) {
        neighbors.get(u).add(v);
        neighbors.get(v).add(u);
    }

    public  void saveEdgesToFile(File file){
            try {
                PrintWriter output = new PrintWriter(file);
                output.print(vertices.size());
                output.println();
                for (int u = 0; u < neighbors.size(); u++) {
                    output.print(u + " ");
                    for (int j = 0; j < neighbors.get(u).size(); j++) {
                        output.print(neighbors.get(u).get(j) + " ");
                    }
                    output.println();
                }
                output.close();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }


    public static class Edge {
        public int u;
        public int v;

        public Edge(int u, int v) {
            this.u = u;
            this.v = v;
        }
    }

    @Override
    public Tree dfs(int v) {
        List<Integer> searchOrder = new ArrayList<Integer>();
        int[] parent = new int[vertices.size()];
        for (int i = 0; i < parent.length; i++)
            parent[i] = -1; // Initialize parent[i] to -1

        boolean[] isVisited = new boolean[vertices.size()];

        dfs(v, parent, searchOrder, isVisited);

        return new Tree(v, parent, searchOrder);
    }

    private void dfs(int v, int[] parent, List<Integer> searchOrder,
                     boolean[] isVisited) {
        searchOrder.add(v);
        isVisited[v] = true; // Vertex v visited

        for (int i : neighbors.get(v)) {
            if (!isVisited[i]) {
                parent[i] = v; // The parent of vertex i is v
                dfs(i, parent, searchOrder, isVisited); // Recursive search
            }
        }
    }

    @Override
    public Tree bfs(int v) {
        List<Integer> searchOrder = new ArrayList<Integer>();
        int[] parent = new int[vertices.size()];
        for (int i = 0; i < parent.length; i++)
            parent[i] = -1;

        java.util.LinkedList<Integer> queue =
                new java.util.LinkedList<Integer>();
        boolean[] isVisited = new boolean[vertices.size()];
        queue.offer(v);
        isVisited[v] = true;

        while (!queue.isEmpty()) {
            int u = queue.poll();
            searchOrder.add(u);
            for (int w : neighbors.get(u)) {
                if (!isVisited[w]) {
                    queue.offer(w);
                    parent[w] = u;
                    isVisited[w] = true;
                }
            }
        }

        return new Tree(v, parent, searchOrder);
    }


    public class Tree {
        private int root;
        private int[] parent;
        private List<Integer> searchOrder;

        public Tree(int root, int[] parent, List<Integer> searchOrder) {
            this.root = root;
            this.parent = parent;
            this.searchOrder = searchOrder;
        }

        public int getRoot() {
            return root;
        }

        public int getParent(int v) {
            return parent[v];
        }

        public List<Integer> getSearchOrder() {
            return searchOrder;
        }

        public int getNumberOfVerticesFound() {
            return searchOrder.size();
        }

        public List<V> getPath(int index) {
            ArrayList<V> path = new ArrayList<V>();

            do {
                path.add(vertices.get(index));
                index = parent[index];
            }
            while (index != -1);

            return path;
        }

        public void printPath(int index) {
            List<V> path = getPath(index);
            System.out.print("A path from " + vertices.get(root) + " to " +
                    vertices.get(index) + ": ");
            for (int i = path.size() - 1; i >= 0; i--)
                System.out.print(path.get(i) + " ");
        }

        public void printTree() {
            System.out.println("Root is: " + vertices.get(root));
            System.out.print("Edges: ");
            for (int i = 0; i < parent.length; i++) {
                if (parent[i] != -1) {
                    // Display an edge
                    System.out.print("(" + vertices.get(parent[i]) + ", " +
                            vertices.get(i) + ") ");
                }
            }
            System.out.println();
        }
    }
}