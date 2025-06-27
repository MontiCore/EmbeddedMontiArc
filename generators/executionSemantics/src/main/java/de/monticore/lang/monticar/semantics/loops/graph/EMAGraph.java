package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;

import java.util.*;
import java.util.stream.Collectors;

public class EMAGraph {
    private Set<EMAComponentInstanceSymbol> vertices;
    private Set<EMAAtomicConnectorInstance> edges;


    public EMAGraph() {
        this.vertices = new HashSet<>();
        this.edges = new HashSet<>();
    }

    public void addSubGraph(EMAGraph subgraph) {
        for (EMAComponentInstanceSymbol vertex : subgraph.getVertices()) {
            addVertex(vertex);
        }
        for (EMAAtomicConnectorInstance edge : subgraph.getEdges()) {
            addEdge(edge);
        }
    }

    public Optional<EMAComponentInstanceSymbol> getVertex(String fullName) {
        List<EMAComponentInstanceSymbol> vertices = getVertices().stream().
                filter(v -> v.getFullName().equals(fullName)).collect(Collectors.toList());

        if (vertices.size() == 1)
            return Optional.of(vertices.get(0));
        return Optional.empty();
    }

    public List<EMAAtomicConnectorInstance> getEdgeWithTargetNode(EMAComponentInstanceSymbol target) {
        if (target == null) return new LinkedList<>();
        List<EMAAtomicConnectorInstance> edges = getEdges().stream().
                filter(v -> v.getTargetComponent() == target).collect(Collectors.toList());

        return edges;
    }

    public List<EMAAtomicConnectorInstance> getEdgesWithSourceNode(EMAComponentInstanceSymbol source) {
        if (source == null) return new LinkedList<>();
        List<EMAAtomicConnectorInstance> edges = getEdges().stream().
                filter(v -> v.getSourceComponent() == source).collect(Collectors.toList());

        return edges;
    }

    public Optional<EMAAtomicConnectorInstance> getEdgeWithTargetPort(EMAPortInstanceSymbol target) {
        if (target == null) return Optional.empty();
        List<EMAAtomicConnectorInstance> edges = getEdges().stream().
                filter(v -> v.getTargetPort() == target).collect(Collectors.toList());

        if (edges.size() == 1)
            return Optional.of(edges.get(0));
        return Optional.empty();
    }

    public List<EMAAtomicConnectorInstance> getEdgesWithSourcePort(EMAPortInstanceSymbol source) {
        if (source == null) return new LinkedList<>();
        List<EMAAtomicConnectorInstance> edges = getEdges().stream().
                filter(v -> v.getSourcePort() == source).collect(Collectors.toList());

        return edges;
    }

    public void addVertex(EMAComponentInstanceSymbol v) {
        vertices.add(v);
    }

    public void addEdge(EMAAtomicConnectorInstance e) {
        edges.add(e);
    }

    public void removeEdge(EMAAtomicConnectorInstance e) {
        edges.remove(e);
    }

    public Set<EMAComponentInstanceSymbol> getVertices() {
        return vertices;
    }

    public Set<EMAAtomicConnectorInstance> getEdges() {
        return edges;
    }
}
