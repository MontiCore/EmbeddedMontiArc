/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.monticar.semantics.construct.*;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopAnalyzer;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopKind;
import de.monticore.lang.monticar.semantics.loops.detection.*;
import de.monticore.lang.monticar.semantics.loops.graph.*;
import de.monticore.lang.monticar.semantics.solver.symbolic.DAESolver;
import de.monticore.lang.monticar.semantics.solver.symbolic.LinearSolver;
import de.monticore.lang.monticar.semantics.solver.symbolic.NonLinearSolver;
import de.monticore.lang.monticar.semantics.solver.symbolic.sympy.SympyDAESolver;
import de.monticore.lang.monticar.semantics.solver.symbolic.sympy.SympyLinearSolver;
import de.monticore.lang.monticar.semantics.solver.symbolic.sympy.SympyNonLinearSolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class Resolver {

    private DAESolver daeSolver;
    private NonLinearSolver nonlinearSolver;
    private LinearSolver linearSolver;
    private boolean solveNumeric = true;
    private EMAComponentInstanceSymbol rootComponent;
    private GlobalScope scope;
    private Replacement replacement;
    private boolean reduceSymbolTableFlag = false; // Reduce symbol table by eliminating unnecessary components

    public Resolver(GlobalScope scope, String rootModel) {
        this.scope = scope;
        this.rootComponent = scope.<EMAComponentInstanceSymbol>resolve(rootModel, EMAComponentInstanceSymbol.KIND).orElse(null);

        this.replacement = new Replacement();

        this.linearSolver = new SympyLinearSolver();
        this.nonlinearSolver = new SympyNonLinearSolver();
        this.daeSolver = new SympyDAESolver();
    }

    public void handleScope() {
        Detection detection = new Detection();
        Set<StrongConnectedComponent> strongConnectedComponents = detection.detectLoops(rootComponent);
        for (StrongConnectedComponent strongConnectedComponent : strongConnectedComponents) {
            LoopAnalyzer analyzer = new LoopAnalyzer();
            analyzer.analyze(strongConnectedComponent);
            if (strongConnectedComponent.getKind().equals(LoopKind.Linear)) {
                handleLinear(strongConnectedComponent);
            } else if (strongConnectedComponent.getKind().equals(LoopKind.LinearDifferencial)) {
                handleDAE(strongConnectedComponent);
            } else if (strongConnectedComponent.getKind().equals(LoopKind.NonLinearDifferencial)) {
                handleDAE(strongConnectedComponent);
            } else if (strongConnectedComponent.getKind().equals(LoopKind.NonLinear)) {
                handleNonLinear(strongConnectedComponent);
            } else if (strongConnectedComponent.getKind().equals(LoopKind.Polynom)) {
                handleNonLinear(strongConnectedComponent);
            } else {
                Log.error("0x907651 not yet supported");
            }
        }

        applyReplacements();
    }

    private Set<String> getVariables(ConnectedComponent component) {
        return component.getPortStatements().keySet().stream().map(p -> p.getFullName()).collect(Collectors.toSet());
    }

    private Set<MathAssignmentExpressionSymbol> getSystem(ConnectedComponent component) {
        return component.getPortStatements().values().stream().collect(Collectors.toSet());
    }

    private void addSymbolicReplacementFromSolutionMap(ConnectedComponent connectedComponent,
                                                       Set<EMAVertex> componentsToReplace,
                                                       Map<String, String> solutionForPort) {
        reduceSymbolTable(connectedComponent, componentsToReplace);
        ReplacementCalculator replacementCalculator = new ReplacementCalculator(replacement);
        replacementCalculator.calculateReplacementsAndGenerateComponents(rootComponent, connectedComponent,
                componentsToReplace, solutionForPort);
    }

    private void reduceSymbolTable(ConnectedComponent connectedComponent, Set<EMAVertex> componentsToReplace) {
        if (reduceSymbolTableFlag) {
            Set<EMAVertex> componentsToRemove = connectedComponent
                    .getAllComponents()
                    .stream()
                    .filter(c -> !componentsToReplace.contains(c))
                    .collect(Collectors.toSet());
            for (EMAVertex emaVertex : componentsToRemove) {
                ConnectedComponentHelper.remove(connectedComponent, emaVertex);
                SymbolTableHelper.removeComponentWithConnectors(scope, emaVertex.getFullName());
            }
        }
    }


    private void handleDAE(ConnectedComponent connectedComponent) {
        Set<MathAssignmentExpressionSymbol> system = getSystem(connectedComponent);
        Set<String> variables = getVariables(connectedComponent);
        Set<EMAVertex> componentsToReplace = calculateComponentsToBreakLoops(connectedComponent);

        Map<String, Double> startValues = new HashMap<>();
        for (EMAPort value : connectedComponent.getGraph().getPortMap().values()) {
            EMAPortInstanceSymbol portInstance = value.getReferencedPort();
            if (portInstance.isInitialGuessPresent()) {
                ASTExpression initialGuess = portInstance.getInitialGuess();
                double val = 0.0;
                if (initialGuess instanceof ASTNumberExpression) {
                    val = (((ASTNumberExpression) initialGuess).getNumberWithUnit()).getNumber().get().doubleValue();
                } else {
                    // TODO
                }
                startValues.put(portInstance.getFullName(), val);
            }
        }

        // Symbolic Solve
        Map<String, String> solutionForPort = daeSolver.solve(system, variables, startValues);

        if (!solutionForPort.isEmpty()) {
            addSymbolicReplacementFromSolutionMap(connectedComponent, componentsToReplace, solutionForPort);
        } else if (solveNumeric) {
            // TODO numeric
            return;
        } else {
            Log.error("TODO");
            return;
        }
    }

    private void handleLinear(ConnectedComponent connectedComponent) {
        Set<MathAssignmentExpressionSymbol> system = getSystem(connectedComponent);
        Set<String> variables = getVariables(connectedComponent);
        Set<EMAVertex> componentsToReplace = calculateComponentsToBreakLoops(connectedComponent);

        Map<String, String> solutionForPort = linearSolver.solve(system, variables);
        if (!solutionForPort.isEmpty()) {
            addSymbolicReplacementFromSolutionMap(connectedComponent, componentsToReplace, solutionForPort);
        } else if (solveNumeric) {
            // TODO numeric
            return ;
        } else {
            Log.error("TODO");
            return ;
        }
    }

    private void handleNonLinear(ConnectedComponent connectedComponent) {
        Set<MathAssignmentExpressionSymbol> system = getSystem(connectedComponent);
        Set<String> variables = getVariables(connectedComponent);
        Set<EMAVertex> componentsToReplace = calculateComponentsToBreakLoops(connectedComponent);

        Map<String, String> solutionForPort = nonlinearSolver.solve(system, variables);
        if (!solutionForPort.isEmpty()) {
            addSymbolicReplacementFromSolutionMap(connectedComponent, componentsToReplace, solutionForPort);
        } else if (solveNumeric) {
            // TODO numeric
            return;
        } else {
            Log.error("TODO");
            return;
        }
    }

    private Set<EMAVertex> calculateComponentsToBreakLoops(ConnectedComponent connectedComponent) {
        Set<EMAVertex> res = new HashSet<>();
        if (connectedComponent instanceof StrongConnectedComponent) {
            for (SimpleCycle simpleCycle : ((StrongConnectedComponent) connectedComponent).getSimpleCycles()) {
                Set<EMAVertex> outs = simpleCycle.getOutports().stream().map(o -> o.getEmaVertex()).collect(Collectors.toSet());
                res.addAll(outs);
            }
        } else if (connectedComponent instanceof SimpleCycle) {
            Set<EMAVertex> outs = connectedComponent.getOutports().stream().map(o -> o.getEmaVertex()).collect(Collectors.toSet());
            res.addAll(outs);
        }
        return res;
    }

    private void applyReplacements() {
        for (ComponentReplacement componentReplacement : replacement.getComponentReplacements()) {
            EMAComponentInstanceSymbol parent = scope.<EMAComponentInstanceSymbol>resolve(
                    componentReplacement.getParentComponent(), EMAComponentInstanceSymbol.KIND).orElse(null);

            // Remove old instance
            SymbolTableHelper.removeComponent(scope, componentReplacement.getPackageName() + "." + componentReplacement.getOldInstanceName());

            // Resolve new instance
            String newChildFullName = NameHelper.toInstanceFullQualifiedName(componentReplacement.getPackageName(),
                    componentReplacement.getType());
            EMAComponentInstanceSymbol newChild = scope.<EMAComponentInstanceSymbol>resolve(
                    newChildFullName, EMAComponentInstanceSymbol.KIND).orElse(null);

            // Rename and repackage
            newChild = InstanceCreator.rename(newChild, componentReplacement.getNewInstanceName());
            newChild.setPackageName(parent.getFullName());
            newChild.setFullName(parent.getFullName() + "." + componentReplacement.getNewInstanceName());
            parent.getSpannedScope().getAsMutableScope().add(newChild);
        }

        for (PortReplacement portReplacement : replacement.getPortReplacements()) {
            EMAComponentInstanceSymbol component = scope.<EMAComponentInstanceSymbol>resolve(
                    portReplacement.getComponent(), EMAComponentInstanceSymbol.KIND).orElse(null);

            EMAPortInstanceSymbol newPort = InstanceCreator.createPortInstanceSymbol(portReplacement.getName(),
                    component.getName(), portReplacement.getType(), portReplacement.isIncoming(),
                    component.getSpannedScope().getAsMutableScope());
            component.getSpannedScope().getAsMutableScope().add(newPort);
        }

        for (ConnectorReplacement connectorReplacement : replacement.getConnectorReplacements()) {
            EMAComponentInstanceSymbol component = scope.<EMAComponentInstanceSymbol>resolve(
                    connectorReplacement.getParentComponent(), EMAComponentInstanceSymbol.KIND).orElse(null);

            EMADynamicConnectorInstanceSymbol newConnector = InstanceCreator.createConnectorInstanceSymbol(connectorReplacement.getSource(), connectorReplacement.getTarget(),
                    component.getName(), component.getSpannedScope().getAsMutableScope());
            component.getSpannedScope().getAsMutableScope().add(newConnector);
        }
    }
}
