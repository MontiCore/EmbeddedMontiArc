/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.math._symboltable.copy.CopyMathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.*;
import de.monticore.lang.monticar.semantics.util.math.NameReplacer;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class LoopAnalyzer {

    public void analyze(StrongConnectedComponent strongConnectedComponent) {
        setPorts(strongConnectedComponent);

        setStatements(strongConnectedComponent);


        // TODO
        analyzeKind(strongConnectedComponent);
    }

    private void setStatements(StrongConnectedComponent strongConnectedComponent) {
        for (EMAVertex component : strongConnectedComponent.getAllComponents()) {
            for (EMAPort outport : component.getOutports()) {
                Optional<MathAssignmentExpressionSymbol> statementForPort = MathStatementCalculator.getStatementForPort(component.getReferencedSymbol(), outport.getName());
                if (statementForPort.isPresent()) {
                    // replace portNames by fullNames
                    Map<String, String> nameMapping = new HashMap<>();
                    nameMapping.put(outport.getName(), outport.getFullName());
                    for (EMAPort inport : component.getInports()) {
                        Optional<EMAEdge> edgeWithTargetPort = strongConnectedComponent.getGraph().getEdgeWithTargetPort(inport);
                        if (!edgeWithTargetPort.isPresent()) {
                            Log.error("0x1544231 no source port for connector");
                        }

                        String sourcePortFullName;
                        if (edgeWithTargetPort.get().getSourceVertex() instanceof EMAPortVertex)
                            sourcePortFullName = edgeWithTargetPort.get().getSourceVertex().getFullName();
                        else
                            sourcePortFullName = edgeWithTargetPort.get().getSourcePort().getFullName();
                        nameMapping.put(inport.getName(), sourcePortFullName);
                    }
                    MathAssignmentExpressionSymbol mathExpressionSymbol = replaceNameInStatement(statementForPort.get(), nameMapping);
                    strongConnectedComponent.addPortStatement(outport, mathExpressionSymbol);
                }
            }
        }
    }

    private MathAssignmentExpressionSymbol replaceNameInStatement(MathAssignmentExpressionSymbol mathExpressionSymbol, Map<String, String> nameMapping) {
        Map<String, String> newNameMapping = new HashMap<>();
        for (Map.Entry<String, String> entry : nameMapping.entrySet()) {
            String newKey = entry.getKey().replace("[","(").replace("]", ")");
            newNameMapping.put(newKey, entry.getValue());
        }
        NameReplacer replacer = new NameReplacer(newNameMapping);
        MathAssignmentExpressionSymbol copy = (MathAssignmentExpressionSymbol) CopyMathExpressionSymbol.copy(mathExpressionSymbol);
        replacer.handle(copy);
        return copy;
    }

    private void setPorts(StrongConnectedComponent strongConnectedComponent) {
        Set<EMAPort> inports = getInports(strongConnectedComponent.getGraph(), strongConnectedComponent.getAllComponents());
        Set<EMAPort> outports = getOutports(strongConnectedComponent.getGraph(), strongConnectedComponent.getAllComponents());

        strongConnectedComponent.setInports(inports);
        strongConnectedComponent.setOutports(outports);

        for (SimpleCycle simpleCycle : strongConnectedComponent.getSimpleCycles()) {
            inports = getInports(simpleCycle.getGraph(), simpleCycle.getAllComponents());
            outports = getOutports(strongConnectedComponent.getGraph(), simpleCycle.getAllComponents());
            simpleCycle.setInports(inports);
            simpleCycle.setOutports(outports);
        }
    }

    private Set<EMAPort> getInports(EMAGraph graph, Collection<EMAVertex> vertices) {
        Set<EMAPort> inports = new HashSet<>();

        for (EMAVertex vertex : vertices) {
            for (EMAPort inport : vertex.getInports()) {
                Optional<EMAEdge> connector = graph.getEdgeWithTargetPort(inport);
                if (!connector.isPresent())
                    inports.add(inport);
                else if (!vertices.contains(connector.get().getSourceVertex()))
                    inports.add(inport);
            }
        }

        return inports;
    }

    private Set<EMAPort> getOutports(EMAGraph graph, Collection<EMAVertex> vertices) {
        Set<EMAPort> outports = new HashSet<>();

        for (EMAVertex vertex : vertices) {
            for (EMAPort outport : vertex.getOutports()) {
                List<EMAEdge> connectors = graph.getEdgesWithSourcePort(outport);
                if (connectors.isEmpty())
                    outports.add(outport);
                else if (!vertices.containsAll(connectors.stream().map(c -> c.getTargetVertex()).collect(Collectors.toSet())))
                    outports.add(outport);
            }
        }

        return outports;
    }

    private void analyzeKind(StrongConnectedComponent strongConnectedComponent) {
        for (SimpleCycle simpleCycle : strongConnectedComponent.getSimpleCycles())
            setKind(simpleCycle);
        setKind(strongConnectedComponent);
    }

    private void setKind(ConnectedComponent component) {
        Set<EMAPort> emaPorts = component.getPortStatements().keySet();
        Set<String> variables = emaPorts.stream().map(s -> s.getFullName()).collect(Collectors.toSet());

        LoopKind currentKind = LoopKind.Default;
        for (EMAPort emaPort : emaPorts) {
            MathAssignmentExpressionSymbol expressionSymbol = component.getPortStatements().get(emaPort);
            CheckKind check = new CheckKind(variables);
            check.handle(expressionSymbol);
            currentKind = LoopKindHelper.combine(currentKind, check.getKind());
        }

        component.setKind(currentKind);
    }
}
