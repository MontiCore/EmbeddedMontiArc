package de.monticore.lang.monticar.semantics.loops.graph;

import java.util.*;
import java.util.stream.Collectors;

public class EMAGraph {
    private Set<EMAVertex> vertices;
    private Set<EMAEdge> edges;

    public Map<String, EMAPort> getPortMap() {
        return portMap;
    }

    private Map<String, EMAPort> portMap = new HashMap<>();


    public EMAGraph() {
        this.vertices = new HashSet<>();
        this.edges = new HashSet<>();
    }

    public void addSubGraph(EMAGraph subgraph) {
        for (EMAVertex vertex : subgraph.getVertices()) {
            addVertex(vertex);
        }
        for (EMAEdge edge : subgraph.getEdges()) {
            addEdge(edge);
        }
    }

    public Optional<EMAVertex> getVertex(String fullName) {
        List<EMAVertex> vertices = getVertices().stream().
                filter(v -> v.getFullName().equals(fullName)).collect(Collectors.toList());

        if (vertices.size() == 1)
            return Optional.of(vertices.get(0));
        return Optional.empty();
    }

    public List<EMAEdge> getEdgeWithTargetNode(EMAVertex target) {
        if (target == null) return new LinkedList<>();
        List<EMAEdge> edges = getEdges().stream().
                filter(v -> v.getTargetVertex() == target).collect(Collectors.toList());

        return edges;
    }

    public List<EMAEdge> getEdgesWithSourceNode(EMAVertex source) {
        if (source == null) return new LinkedList<>();
        List<EMAEdge> edges = getEdges().stream().
                filter(v -> v.getSourceVertex() == source).collect(Collectors.toList());

        return edges;
    }

    public Optional<EMAEdge> getEdgeWithTargetPort(EMAPort target) {
        if (target == null) return Optional.empty();
        List<EMAEdge> edges = getEdges().stream().
                filter(v -> v.getTargetPort() == target).collect(Collectors.toList());

        if (edges.size() == 1)
            return Optional.of(edges.get(0));
        return Optional.empty();
    }

    public List<EMAEdge> getEdgesWithSourcePort(EMAPort source) {
        if (source == null) return new LinkedList<>();
        List<EMAEdge> edges = getEdges().stream().
                filter(v -> v.getSourcePort() == source).collect(Collectors.toList());

        return edges;
    }

    public void addVertex(EMAVertex v) {
        vertices.add(v);
        for (EMAPort outport : v.getOutports()) {
            portMap.put(outport.getFullName(), outport);
        }
        for (EMAPort inport : v.getInports()) {
            portMap.put(inport.getFullName(), inport);
        }
    }

    public void removeVertex(EMAVertex v) {
        vertices.remove(v);
        for (EMAPort outport : v.getOutports()) {
            portMap.remove(outport.getFullName(), outport);
        }
        for (EMAPort inport : v.getInports()) {
            portMap.remove(inport.getFullName(), inport);
        }
    }

    public void removePortVertex(EMAPortVertex v) {
        vertices.remove(v);
    }

    public void addEdge(EMAEdge e) {
        edges.add(e);
    }

    public void removeEdge(EMAEdge e) {
        edges.remove(e);
    }

    public Set<EMAVertex> getVertices() {
        return vertices;
    }

    public Set<EMAEdge> getEdges() {
        return edges;
    }
}
