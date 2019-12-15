
import java.util.List;
import java.util.LinkedList;
import java.util.Set;
import java.util.Map;
import java.util.HashSet;
import java.util.HashMap;
import java.util.Random;
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.Collections;
import java.util.Comparator;

import java.util.stream.Collectors;


public class PathFinder<V> {

    private DirectedGraph<V> graph;
    private long startTimeMillis;


    public PathFinder(DirectedGraph<V> graph) {
        this.graph = graph;
    }


    public class Result<V> {
        public final boolean success;
        public final V start;
        public final V goal;
        public final double cost;
        public final List<V> path;
        public final int visitedNodes;
        public final double elapsedTime;

        public Result(boolean success, V start, V goal, double cost, List<V> path, int visitedNodes) {
            this.success = success;
            this.start = start;
            this.goal = goal;
            this.cost = cost;
            this.path = path;
            this.visitedNodes = visitedNodes;
            this.elapsedTime = (System.currentTimeMillis() - startTimeMillis) / 1000.0;
        }

        public String toString() {
            String s = "";
            s += String.format("Visited nodes: %d\n", visitedNodes);
            s += String.format("Elapsed time: %.1f seconds\n", elapsedTime);
            if (success) {
                s += String.format("Total cost from %s -> %s: %s\n", start, goal, cost);
                s += "Path: " + path.stream().map(x -> x.toString()).collect(Collectors.joining(" -> "));
            } else {
                s += String.format("No path found from %s", start);
            }
            return s;
        }
    }


    public Result<V> search(String algorithm, V start, V goal) {
        startTimeMillis = System.currentTimeMillis();
        switch (algorithm) {
        case "random":   return searchRandom(start, goal);
        case "dijkstra": return searchDijkstra(start, goal);
        case "astar":    return searchAstar(start, goal);
        }
        throw new IllegalArgumentException("Unknown search algorithm: " + algorithm);
    }

    public Result<V> searchRandom(V start, V goal) {
        int visitedNodes = 0;
        LinkedList<V> path = new LinkedList<>();
        double cost = 0.0;
        Random random = new Random();

        V current = start;
        path.add(current);
        while (current != null) {
            visitedNodes++;
            if (current.equals(goal)) {
                return new Result<>(true, start, current, cost, path, visitedNodes);
            }

            List<DirectedEdge<V>> neighbours = graph.outgoingEdges(start);
            if (neighbours == null || neighbours.size() == 0) {
                break;
            } else {
                DirectedEdge<V> edge = neighbours.get(random.nextInt(neighbours.size()));
                cost += edge.weight();
                current = edge.to();
                path.add(current);
            }
        }
        return new Result<>(false, start, null, -1, null, visitedNodes);
    }


    public Result<V> searchDijkstra(V start, V goal) {
        Set<V> discovered = new HashSet<>();
        Map<V, Double> distTo = new HashMap<>();
        Map<V, V> nodeTo = new HashMap<>();
        Queue<V> pq = new PriorityQueue<>(1, new Comparator<V>() { /* Breaking data structure invariant? */
            @Override
            public int compare(V o1, V o2) {
                return Double.compare(distTo.get(o1), distTo.get(o2));
            }
        });

        distTo.put(start, 0.0);
        nodeTo.put(start, null);
        pq.add(start);

        while (!pq.isEmpty()) {
            V v = pq.remove();

            if (!discovered.contains(v)) {
                discovered.add(v);

                if (v.equals(goal)) {
                    /* Build path */
                    List<V> path = new LinkedList<>();
                    for (V u = goal; u != null; u = nodeTo.get(u))
                        path.add(u);
                    Collections.reverse(path);
                    return new Result<>(true, start, goal, distTo.get(goal), path, discovered.size());
                }

                for (DirectedEdge<V> e: graph.outgoingEdges(v)) {
                    V w = e.to();
                    double distAlt = distTo.get(v) + e.weight();
                    /* If we haven't seen the node yet or relax */
                    if (!distTo.containsKey(w) || distAlt < distTo.get(w)) {
                        distTo.put(w, distAlt);
                        nodeTo.put(w, v);
                        pq.add(w); /* Because we have no decrease priority */
                    }
                }
            }
        }

        return new Result<>(false, start, null, -1, null, discovered.size());
    }

    public Result<V> searchAstar(V start, V goal) {
        Set<V> discovered = new HashSet<>();
        Map<V, V> nodeTo = new HashMap<>();     /* The node before each node V in the shortest path */
        Map<V, Double> gScore = new HashMap<>();
        Map<V, Double> fScore = new HashMap<>();
        Queue<V> pq = new PriorityQueue<>(1, new Comparator<V>() { /* Breaking data structure invariant? */
            @Override
            public int compare(V o1, V o2) {
                return Double.compare(fScore.get(o1), fScore.get(o2));
            }
        });

        nodeTo.put(start, null);
        gScore.put(start, 0.0);
        fScore.put(start, graph.guessCost(start, goal));
        pq.add(start);

        while (!pq.isEmpty()) {
            V v = pq.remove();

            if (!discovered.contains(v)) {
                discovered.add(v);

                if (v.equals(goal)) {
                    /* Build path */
                    List<V> path = new LinkedList<>();
                    for (V u = goal; u != null; u = nodeTo.get(u))
                        path.add(u);
                    Collections.reverse(path);
                    return new Result<>(true, start, goal, gScore.get(goal), path, discovered.size());
                }

                for (DirectedEdge<V> e: graph.outgoingEdges(v)) {
                    V w = e.to();

                    double distAlt = gScore.get(v) + e.weight();
                    /* If we haven't seen the node yet or relax */
                    if (!fScore.containsKey(w) || distAlt < gScore.get(w)) {
                        gScore.put(w, distAlt);
                        fScore.put(w, distAlt + graph.guessCost(w, goal));
                        nodeTo.put(w, v);
                        pq.add(w); /* Because we have no decrease priority */
                    }
                }
            }
        }

        return new Result<>(false, start, null, -1, null, discovered.size());
    }

}
