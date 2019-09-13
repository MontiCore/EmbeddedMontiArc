/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.Vector;

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.MapData;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;

public class Pathfinding {
    MapData map;
    HashMap<Long, Vertex> vertices;

    private static class Edge {
        long neighbor_id;
        double cost;

        public Edge(long neighbor_id, double cost){
            this.neighbor_id = neighbor_id;
            this.cost = cost;
        }
    }

    private static class Vertex {
        boolean visited;
        List<Edge> neighbors;
        long predecessor;
        double best_cost;
        Point2D pos;
        int path_node_count;

        public Vertex(Point2D pos){
            this.pos = pos;
            neighbors = new LinkedList<>();
        }

        public void init(){
            path_node_count = 0;
            visited = false;
            predecessor = -1;
            best_cost = Double.POSITIVE_INFINITY;
        }
    }

    public Pathfinding(MapData map){
        HashSet<String> allowed_ways = new HashSet<>();
        allowed_ways.add("motorway");
        allowed_ways.add("trunk");
        allowed_ways.add("primary");
        allowed_ways.add("secondary");
        allowed_ways.add("tertiary");
        allowed_ways.add("unclassified");
        allowed_ways.add("residential");
        allowed_ways.add("motorway_link");
        allowed_ways.add("trunk_link");
        allowed_ways.add("primary_link");
        allowed_ways.add("secondary_link");
        allowed_ways.add("tertiary_link");
        allowed_ways.add("living_street");
        allowed_ways.add("service");
        allowed_ways.add("pedestrian");
        allowed_ways.add("track");
        allowed_ways.add("raceway");
        this.map = map;
        List<Long> used_ways = new LinkedList<>();
        HashSet<Long> used_vertices = new LinkedHashSet<>();

        for (Map.Entry<Long, MapData.Way> c : map.ways.entrySet()){
            MapData.Way way = c.getValue();
            if (way.driveable){
                used_ways.add(c.getKey());
                for (Long nid : way.nodes){
                    if (!used_vertices.contains(nid)) 
                        used_vertices.add(nid);
                }
            }
        }

        vertices = new HashMap<>();
        for (Long nid : used_vertices){
            vertices.put(nid, new Vertex(map.nodes.get(nid).point));
        }

        for (Long wid : used_ways){
            MapData.Way way = map.ways.get(wid);
            //Get oneway property
            boolean oneway = false;
            String ow = way.tags.get("oneway");
            if (ow != null && ow.equals("yes"))
                oneway = true;
            
            double max_speed = 50; //Km/H
            String ms = way.tags.get("maxspeed");
            if (ms != null)
                max_speed = Double.parseDouble(ms);
                
            double time_factor = 3.6 / max_speed;

            MapData.Node prev = null;
            Vertex prev_vert = null;
            long prev_id = 0;
            for (Long nid : way.nodes){
                MapData.Node node = map.nodes.get(nid);
                Vertex vert = vertices.get(nid);
                if (node != null){
                    if (prev != null){
                        double cost = node.point.distance(prev.point) * time_factor;
                        prev_vert.neighbors.add(new Edge(nid, cost));
                        if (!oneway){
                            vert.neighbors.add(new Edge(prev_id, cost));
                        }
                    }
                    prev = node;
                    prev_vert = vert;
                    prev_id = nid;
                }
            }
        }
    }

    private static class EdgeComparator implements Comparator {
        @Override
        public int compare(Object o1, Object o2) {
            if (((Edge)o1).cost < ((Edge)o2).cost) return -1;
            if (((Edge)o1).cost > ((Edge)o2).cost) return -1;
            return 0;
        }
    }

    public Point2D[] find_shortest_path(Point2D start_coords, Point2D target_coords) throws Exception {
        //Get nearest nodes
        double best_start = Double.MAX_VALUE, best_target = Double.MAX_VALUE;
        long nearest_start = -1, nearest_target = -1;

        
        //Init vertices (= reset color/predecessor/...) at the same time
        for (Map.Entry<Long, Vertex> c : vertices.entrySet()){
            long id = c.getKey();
            Vertex v = c.getValue();
            double dist_start = v.pos.distance(start_coords);
            if (dist_start < best_start){
                best_start = dist_start;
                nearest_start = id;
            }
            double dist_target = v.pos.distance(target_coords);
            if (dist_target < best_target){
                best_target = dist_target;
                nearest_target = id;
            }

            v.init();
        }
        if (nearest_start == -1 || nearest_target == -1) throw new Exception("Error finding nearest node for trajectory.");
        if (nearest_start == nearest_target) throw new Exception("Found trajectory has same start and end.");
        
        
        //Dijkstra !

        //Neighbor map
        PriorityQueue<Edge> neighbors = new PriorityQueue<Edge>(new EdgeComparator());

        //First node
        Vertex v_start = vertices.get(nearest_start);
        v_start.visited = true;
        v_start.best_cost = 0;
        v_start.predecessor = nearest_start;
        v_start.path_node_count = 1;
        for (Edge e : v_start.neighbors){
            neighbors.add(e);
            Vertex v = vertices.get(e.neighbor_id);
            v.best_cost = e.cost;
            v.predecessor = nearest_start;
            v.path_node_count = 2;
        }

        while(!neighbors.isEmpty()){
            Edge next_edge = neighbors.poll();
            Vertex v = vertices.get(next_edge.neighbor_id);
            v.visited = true;
            if (next_edge.neighbor_id == nearest_target){
                break;
            }
            for (Edge e : v.neighbors){
                Vertex n_v = vertices.get(e.neighbor_id);
                if (!n_v.visited){
                    double new_cost = v.best_cost + e.cost;
                    if (new_cost < n_v.best_cost){
                        n_v.best_cost = new_cost;
                        n_v.predecessor = next_edge.neighbor_id;
                        n_v.path_node_count = v.path_node_count + 1;
                    }
                    neighbors.offer(e);
                }
            }
        }

        Vertex v_target = vertices.get(nearest_target);
        if (!v_target.visited) throw new Exception("Could not find path between start and target coordinates.");

        Point2D[] path = new Point2D[v_target.path_node_count];
        int pos = v_target.path_node_count - 1;
        long current_id = nearest_target;
        do {
            Vertex v = vertices.get(current_id);
            path[pos] = v.pos;
            pos--;
            current_id = v.predecessor;
        } while (current_id != nearest_start);
        path[0] = v_start.pos;
        return path;
    }

}