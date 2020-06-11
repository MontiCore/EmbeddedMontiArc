/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.ast.ASTNode;
import de.monticore.commonexpressions._ast.ASTDivideExpression;
import de.monticore.commonexpressions._ast.ASTMinusExpression;
import de.monticore.commonexpressions._ast.ASTMultExpression;
import de.monticore.commonexpressions._ast.ASTPlusExpression;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.EmbeddedMontiArcMathMill;
import de.monticore.lang.math._ast.*;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._visitor.MathVisitor;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.*;
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
                Optional<ASTMathAssignmentStatement> statementForPort = getStatementForPort(component.getReferencedSymbol(), outport.getName());
                if (statementForPort.isPresent()) {
                    // replace portNames by fullNames
                    statementForPort.get().setName(outport.getFullName());
                    for (EMAPort inport : component.getInports()) {
                        Optional<EMAEdge> edgeWithTargetPort = strongConnectedComponent.getGraph().getEdgeWithTargetPort(inport);
                        if (!edgeWithTargetPort.isPresent()) {
                            Log.error ("0x1544231 no source port for connector");
                        }

                        String sourcePort;
                        if (edgeWithTargetPort.get().getSourceVertex() instanceof EMAPortVertex)
                            sourcePort = edgeWithTargetPort.get().getSourceVertex().getFullName();
                        else
                            sourcePort = edgeWithTargetPort.get().getSourcePort().getFullName();
                        statementForPort.get().accept(new NameReplacer(inport.getName(), sourcePort));
                    }
                    strongConnectedComponent.addPortStatement(outport, statementForPort.get());
                }
            }
         }
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

    private Optional<ASTMathAssignmentStatement> getStatementForPort(EMAComponentInstantiationSymbol component, String portName) {
        List<ASTMathAssignmentStatement> statements = new LinkedList<>();
        Optional<MathStatementsSymbol> mathStatements = component.getComponentType().getReferencedComponent().get().getSpannedScope().<MathStatementsSymbol>resolve("MathStatements", MathStatementsSymbol.KIND);
        if (!mathStatements.isPresent())
            return Optional.empty();
        else {
            for (MathExpressionSymbol statement : mathStatements.get().getMathExpressionSymbols()) {
                ASTNode astNode = statement.getAstNode().get();
                if (astNode instanceof ASTMathAssignmentStatement)
                    statements.add((ASTMathAssignmentStatement) astNode);
                else if (astNode instanceof ASTMathAssignmentDeclarationStatement) {
                    ASTMathAssignmentDeclarationStatement declaration = (ASTMathAssignmentDeclarationStatement) astNode;
                    ASTMathAssignmentStatement newStatement =
                            EmbeddedMontiArcMathMill.mathAssignmentStatementBuilder()
                                    .setName(declaration.getName())
                                    .setMathAssignmentOperator(declaration.getMathAssignmentOperator())
                                    .setExpression(declaration.getExpression())
                                    .setEnclosingScope(declaration.getEnclosingScope())
                                    .set_SourcePositionStart(declaration.get_SourcePositionStart())
                                    .set_SourcePositionEnd(declaration.get_SourcePositionEnd())
                                    .build();
                    statements.add(newStatement);
                } else
                    Log.error("0x654987 not yet supported");
            }
        }

        ListIterator<ASTMathAssignmentStatement> iterator = statements.listIterator(statements.size());
        ASTMathAssignmentStatement res = null;

        while (iterator.hasPrevious()) {
            ASTMathAssignmentStatement previous = iterator.previous();
            if (previous.getName().equals(portName)) {
                if (res == null)
                    res = previous.deepClone();
            } else {
                if (res != null)
                    if (res.getExpression() instanceof ASTNameExpression) {
                        if (((ASTNameExpression) res.getExpression()).getName().equals(previous.getName()))
                            res.setExpression(previous.getExpression());
                    } else
                        res.getExpression().accept(new ExpressionReplacer(previous.getName(), previous.getExpression()));
            }
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
