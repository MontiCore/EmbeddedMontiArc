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
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.*;
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
                Optional<MathAssignmentExpressionSymbol> statementForPort = getStatementForPort(component.getReferencedSymbol(), outport.getName());
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
            simpleCycle.setKind(LoopKind.QuadraticLinear);
        }
        strongConnectedComponent.setKind(LoopKind.QuadraticLinear);

    }

    private Optional<MathAssignmentExpressionSymbol> getStatementForPort(EMAComponentInstanceSymbol component, String portName) {
        // Work on symbols because the the symbol table will be recreated either way
        Optional<MathStatementsSymbol> mathStatements = component.getSpannedScope().<MathStatementsSymbol>resolve("MathStatements", MathStatementsSymbol.KIND);
        if (!mathStatements.isPresent())
            return Optional.empty();


        MathAssignmentExpressionSymbol res = null;
        ListIterator<MathExpressionSymbol> iter = mathStatements.get().getMathExpressionSymbols()
                .listIterator(mathStatements.get().getMathExpressionSymbols().size());
        while (iter.hasPrevious()) {
            MathExpressionSymbol statement = iter.previous();
            if (statement.isAssignmentExpression()) {
                MathAssignmentExpressionSymbol assignmentStatement = (MathAssignmentExpressionSymbol) statement;

                if (assignmentStatement.getNameOfMathValue().equals(portName)) {
                    res = assignmentStatement;
                } else if (res != null) {
                    // TODO replace
                    res = res;
                }
            } else
                Log.error("0x654987 not yet supported");
        }

        return Optional.ofNullable(res);
    }

    private class ExpressionReplacer implements MathVisitor {

        // TODO implement all AST-Expressions visit

        private final String toSubstitue;
        private final ASTExpression expression;

        private ExpressionReplacer(String toSubstitute, ASTExpression expression) {
            this.toSubstitue = toSubstitute;
            this.expression = expression;
        }

        private boolean isNameExpression(ASTExpression nameExpression) {
            if (nameExpression instanceof ASTNameExpression)
                if (((ASTNameExpression) nameExpression).getName().equals(toSubstitue))
                    return true;
            return false;
        }

        @Override
        public void visit(ASTMathArithmeticPowerOfExpression node) {
            if (isNameExpression(node.getLeftExpression())) {
                node.setLeftExpression(expression);
            }
            if (isNameExpression(node.getRightExpression())) {
                node.setRightExpression(expression);
            }
        }

        @Override
        public void visit(ASTIncSuffixExpression node) {
            if (isNameExpression(node.getExpression())) {
                node.setExpression(expression);
            }
        }

        @Override
        public void visit(ASTDecSuffixExpression node) {
            if (isNameExpression(node.getExpression())) {
                node.setExpression(expression);
            }
        }

        @Override
        public void visit(ASTMultExpression node) {
            if (isNameExpression(node.getLeftExpression())) {
                node.setLeftExpression(expression);
            }
            if (isNameExpression(node.getRightExpression())) {
                node.setRightExpression(expression);
            }
        }

        @Override
        public void visit(ASTDivideExpression node) {
            if (isNameExpression(node.getLeftExpression())) {
                node.setLeftExpression(expression);
            }
            if (isNameExpression(node.getRightExpression())) {
                node.setRightExpression(expression);
            }
        }

        @Override
        public void visit(ASTPlusExpression node) {
            if (isNameExpression(node.getLeftExpression())) {
                node.setLeftExpression(expression);
            }
            if (isNameExpression(node.getRightExpression())) {
                node.setRightExpression(expression);
            }
        }

        @Override
        public void visit(ASTMinusExpression node) {
            if (isNameExpression(node.getLeftExpression())) {
                node.setLeftExpression(expression);
            }
            if (isNameExpression(node.getRightExpression())) {
                node.setRightExpression(expression);
            }
        }
    }
}
