package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.*;

import java.util.*;

public class EMAGraphTransformation {


    public EMAGraph transform(EMAComponentInstanceSymbol component) {
        EMAGraph graph = new EMAGraph();

        boolean isAtomic = true;

        for (EMAComponentInstanceSymbol sub : component.getSubComponents()) {
            isAtomic = false;
            EMAGraph subGraph = transform(sub);
            graph.addSubGraph(subGraph);
        }

        Set<EMAPortVertex> newPorts = new HashSet<>();
        for (EMAPortInstanceSymbol port: component.getPortInstanceList()) {
            EMAPort newEmaPort = new EMAPort(port);
            List<EMAPort> inports = new LinkedList<>();
            List<EMAPort> outports = new LinkedList<>();
            outports.add(newEmaPort);
            inports.add(newEmaPort);
            EMAPortVertex newPort = new EMAPortVertex(null, port.getName(), port.getFullName(), inports, outports);
            newEmaPort.setEmaVertex(newPort);
            graph.addVertex(newPort);
            newPorts.add(newPort);
        }

        if (isAtomic) {
            EMAVertex newVertex = EMAVertex.create(component);
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
            for (EMAConnectorInstanceSymbol connector: component.getConnectorInstances()) {
                Optional<EMAVertex> source = graph.getVertex(connector.getSourcePort().getFullName());
                Optional<EMAVertex> target = graph.getVertex(connector.getTargetPort().getFullName());
                EMAPortEdge newEdge = new EMAPortEdge(source.get(), target.get()
                , source.get().getOutports().get(0), target.get().getInports().get(0));
                graph.addEdge(newEdge);
            }
            removeInnerPorts(graph, newPorts);
        }

        removeInnerPorts(graph, newPorts);

        return graph;
    }

    private void removeInnerPorts(EMAGraph graph, Set<EMAPortVertex> newPorts) {
        Set<EMAPortVertex> portsToRemove = new HashSet<>();

        for (EMAVertex vertex: graph.getVertices()) {
            if (vertex instanceof EMAPortVertex && !newPorts.contains(vertex)) {
                List<EMAEdge> edgesWithTargetNode = graph.getEdgeWithTargetNode(vertex);
                Optional<EMAEdge> from = Optional.empty();
                if (edgesWithTargetNode.size() == 1)
                    from = Optional.ofNullable(edgesWithTargetNode.get(0));
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
                portsToRemove.add((EMAPortVertex) vertex);
            }
        }
        for (EMAPortVertex v: portsToRemove) {
            graph.removePortVertex(v);
        }
    }

}
