/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.commonexpressions._ast.ASTDivideExpression;
import de.monticore.commonexpressions._ast.ASTMinusExpression;
import de.monticore.commonexpressions._ast.ASTMultExpression;
import de.monticore.commonexpressions._ast.ASTPlusExpression;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._ast.*;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._visitor.MathVisitor;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.*;
import de.monticore.lang.monticar.semantics.util.math.CopyMathSymbol;
import de.monticore.lang.monticar.semantics.util.math.NameReplacer;
import de.se_rwth.commons.logging.Log;
import org.apache.poi.ss.formula.functions.Na;

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
                    statementForPort.get().setNameOfMathValue(outport.getFullName());
                    for (EMAPort inport : component.getInports()) {
                        Optional<EMAEdge> edgeWithTargetPort = strongConnectedComponent.getGraph().getEdgeWithTargetPort(inport);
                        if (!edgeWithTargetPort.isPresent()) {
                            Log.error("0x1544231 no source port for connector");
                        }

                        String sourcePort;
                        if (edgeWithTargetPort.get().getSourceVertex() instanceof EMAPortVertex)
                            sourcePort = edgeWithTargetPort.get().getSourceVertex().getFullName();
                        else
                            sourcePort = edgeWithTargetPort.get().getSourcePort().getFullName();
                        replaceNameInStatement(statementForPort.get(), sourcePort, inport.getName());
                    }
                    strongConnectedComponent.addPortStatement(outport, statementForPort.get());
                }
            }
        }
    }

    private void replaceNameInStatement(MathExpressionSymbol mathExpressionSymbol, String newName, String oldName) {
        oldName = oldName.replace("[","(").replace("]", ")");
        NameReplacer replacer = new NameReplacer(newName, oldName);
        replacer.handle(mathExpressionSymbol);
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
        // TODO
        for (SimpleCycle simpleCycle : strongConnectedComponent.getSimpleCycles()) {
            if (isLinear(simpleCycle))
                simpleCycle.setKind(LoopKind.QuadraticLinear);
        }
        if (isLinear(strongConnectedComponent))
            strongConnectedComponent.setKind(LoopKind.QuadraticLinear);

    }

    private boolean isLinear(ConnectedComponent system) {
        Set<EMAPort> emaPorts = system.getPortStatements().keySet();
        Set<String> variables = emaPorts.stream().map(s -> s.getFullName()).collect(Collectors.toSet());

        for (EMAPort emaPort : emaPorts) {
            MathAssignmentExpressionSymbol expressionSymbol = system.getPortStatements().get(emaPort);
            CheckLinear check = new CheckLinear(variables);
            check.handle(expressionSymbol);
            if (!check.getResult())
                return false;
        }

        return true;
    }
}
