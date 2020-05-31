package de.monticore.lang.monticar.semantics.loops.graph;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class EMAGraph {
    private Set<EMAVertex> vertices;
    private Set<EMAEdge> edges;

    public EMAGraph() {
        this.vertices = new HashSet<>();
        this.edges = new HashSet<>();
    }

    public void addSubGraph(EMAGraph subgraph) {
        vertices.addAll(subgraph.getVertices());
        edges.addAll(subgraph.getEdges());
    }

    public Optional<EMAVertex> getVertex(String fullName) {
        List<EMAVertex> vertices = getVertices().stream().
                filter(v -> v.getFullName().equals(fullName)).collect(Collectors.toList());

        if (vertices.size() == 1)
            return Optional.of(vertices.get(0));
        return Optional.empty();
    }

    public Optional<EMAEdge> getEdgeWithTargetNode(EMAVertex target) {
        List<EMAEdge> edges = getEdges().stream().
                filter(v -> v.getTargetVertex() == target).collect(Collectors.toList());

        if (edges.size() == 1)
            return Optional.of(edges.get(0));
        return Optional.empty();
    }

    public List<EMAEdge> getEdgesWithSourceNode(EMAVertex source) {
        List<EMAEdge> edges = getEdges().stream().
                filter(v -> v.getSourceVertex() == source).collect(Collectors.toList());

        return edges;
    }

    public Optional<EMAEdge> getEdgeWithTargetPort(EMAPort target) {
        List<EMAEdge> edges = getEdges().stream().
                filter(v -> v.getTargetPort() == target).collect(Collectors.toList());

        if (edges.size() == 1)
            return Optional.of(edges.get(0));
        return Optional.empty();
    }

    public List<EMAEdge> getEdgesWithSourcePort(EMAPort source) {
        List<EMAEdge> edges = getEdges().stream().
                filter(v -> v.getSourcePort() == source).collect(Collectors.toList());

        return edges;
    }

    public void addVertex(EMAVertex v) {
        vertices.add(v);
    }

    public void removeVertex(EMAVertex v) {
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
