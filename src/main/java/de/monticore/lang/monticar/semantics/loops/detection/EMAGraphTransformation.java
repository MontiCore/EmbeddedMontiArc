package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.*;

import java.util.*;

public class EMAGraphTransformation {


    public EMAGraph transform(EMAComponentSymbol component) {
        EMAGraph graph = new EMAGraph();

        String name = component.getName();

        for (EMAComponentInstantiationSymbol sub : component.getSubComponents()) {
            EMAGraph subGraph = transform(sub, name);
            graph.addSubGraph(subGraph);
        }

        Set<EMAPortVertex> newPorts = new HashSet<>();
        for (EMAPortSymbol port: component.getPortsList()) {
            EMAPortVertex newPort = new EMAPortVertex(port.getName(), name + "." + port.getName());
            graph.addVertex(newPort);
            newPorts.add(newPort);
        }

        for (EMAConnectorSymbol connector: component.getConnectors()) {
            EMAVertex source = graph.getVertex(name + "." + connector.getSource()).get();
            EMAVertex target = graph.getVertex(name + "." + connector.getTarget()).get();
            EMAPortEdge newEdge = new EMAPortEdge(source, target);
            graph.addEdge(newEdge);
        }
        removeInnerPorts(graph, newPorts);

        return graph;
    }

    private EMAGraph transform(EMAComponentInstantiationSymbol component, String parentFullName) {
        EMAGraph graph = new EMAGraph();
        String fullName = parentFullName + "." + component.getName();

        boolean isAtomic = true;

        for (EMAComponentInstantiationSymbol sub : getSubComponents(component)) {
            isAtomic = false;
            EMAGraph subGraph = transform(sub, fullName);
            graph.addSubGraph(subGraph);
        }

        // addPorts
        Set<EMAPortVertex> newPorts = new HashSet<>();
        for (EMAPortSymbol port: getPorts(component)) {
            EMAPortVertex newPort = new EMAPortVertex(port.getName(), fullName + "." + port.getName());
            graph.addVertex(newPort);
            newPorts.add(newPort);
        }

        if (isAtomic) {
            EMAVertex newVertex = EMAVertex.create(component, parentFullName);
            graph.addVertex(newVertex);

            for (EMAPort inport: newVertex.getInports()) {
                EMAVertex inportVertex = graph.getVertex(inport.getFullName()).get();
                graph.addEdge(new EMAEdge(inportVertex, newVertex, null, inport));
            }
            for (EMAPort outport: newVertex.getOutports()) {
                EMAVertex outportVertex = graph.getVertex(outport.getFullName()).get();
                graph.addEdge(new EMAEdge(newVertex, outportVertex, outport, null));
            }
        } else {
            for (EMAConnectorSymbol connector: getConnectors(component)) {
                Optional<EMAVertex> source = graph.getVertex(fullName + "." + connector.getSource());
                Optional<EMAVertex> target = graph.getVertex(fullName + "." + connector.getTarget());
                EMAPortEdge newEdge = new EMAPortEdge(source.get(), target.get());
                    graph.addEdge(newEdge);
            }
            removeInnerPorts(graph, newPorts);
        }

        return graph;
    }

    private void removeInnerPorts(EMAGraph graph, Set<EMAPortVertex> newPorts) {
        Set<EMAVertex> portsToRemove = new HashSet<>();

        for (EMAVertex vertex: graph.getVertices()) {
            if (vertex instanceof EMAPortVertex && !newPorts.contains(vertex)) {
                Optional<EMAEdge> from = graph.getEdgeWithTargetNode(vertex);
                List<EMAEdge> tos = graph.getEdgesWithSourceNode(vertex);

                if (from.isPresent()) {
                    for (EMAEdge to: tos) {
                        EMAVertex sourceVertex = from.get().getSourceVertex();
                        EMAVertex targetVertex = to.getTargetVertex();
                        EMAPort sourcePort = from.get().getSourcePort();
                        EMAPort targetPort = to.getTargetPort();
                        EMAEdge newEdge = new EMAEdge(sourceVertex, targetVertex, sourcePort, targetPort);
                        graph.addEdge(newEdge);
                    }
                    graph.removeEdge(from.get());
                }
                for (EMAEdge to: tos) {
                    graph.removeEdge(to);
                }
                portsToRemove.add(vertex);
            }
        }
        for (EMAVertex v: portsToRemove) {
            graph.removeVertex(v);
        }
    }

    private Collection<EMAComponentInstantiationSymbol> getSubComponents(EMAComponentInstantiationSymbol component) {
        return component.getComponentType().getSubComponents();
    }

    private Collection<EMAConnectorSymbol> getConnectors(EMAComponentInstantiationSymbol component) {
        return component.getComponentType().getReferencedSymbol().getConnectors();
    }

    private Collection<EMAPortSymbol> getPorts(EMAComponentInstantiationSymbol component) {
        return component.getComponentType().getReferencedSymbol().getPortsList();
    }

}
