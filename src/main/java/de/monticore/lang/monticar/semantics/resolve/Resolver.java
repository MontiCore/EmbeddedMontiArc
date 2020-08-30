/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.monticar.semantics.construct.*;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopAnalyzer;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopKind;
import de.monticore.lang.monticar.semantics.loops.detection.Detection;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.*;
import de.monticore.lang.monticar.semantics.solver.linearsolver.LinearEquationSystemSolver;
import de.monticore.lang.monticar.semantics.solver.linearsolver.MathEclipseSolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class Resolver {

    private boolean solveNumeric;
    private String rootModel;
    private EMAComponentInstanceSymbol rootComponent;

    public Resolver(String rootModel) {
        this.rootModel = rootModel;
    }

    public void handleScope(GlobalScope scope) {
        rootComponent = scope.<EMAComponentInstanceSymbol>resolve(rootModel, EMAComponentInstanceSymbol.KIND).orElse(null);
//        fix(component);

        Detection detection = new Detection();
        Set<StrongConnectedComponent> strongConnectedComponents = detection.detectLoops(rootComponent);
        Replacement replacements = null;

        for (StrongConnectedComponent strongConnectedComponent : strongConnectedComponents) {
            LoopAnalyzer analyzer = new LoopAnalyzer();
            analyzer.analyze(strongConnectedComponent);
            if (strongConnectedComponent.getKind().equals(LoopKind.QuadraticLinear)) {
                replacements = handleLinear(strongConnectedComponent);
            } else {
                Log.error("0x907651 not yet supported");
            }
        }

        if (replacements != null) {
            applyReplacements(scope, replacements);
        }
    }

    private Replacement handleLinear(StrongConnectedComponent strongConnectedComponent) {
        MathEclipseSolver msolver = new MathEclipseSolver();
        LinearEquationSystemSolver solver = new LinearEquationSystemSolver(msolver, msolver);

        Set<MathAssignmentExpressionSymbol> system = new HashSet<>();
        Set<String> variables = new HashSet<>();

        for (EMAPort emaPort : strongConnectedComponent.getPortStatements().keySet()) {
            variables.add(emaPort.getFullName());
            MathAssignmentExpressionSymbol statement = strongConnectedComponent.getPortStatements().get(emaPort);
            system.add(statement);
        }

        Map<String, String> solutionForPort = solver.solve(system, variables);

        Set<EMAVertex> componentsToReplace = calculateComponentsToBreakLoops(strongConnectedComponent);

        ReplacementCalculator replacementCalculator = new ReplacementCalculator();
        replacementCalculator.calculateReplacementsAndGenerateComponents(rootComponent, strongConnectedComponent,
                componentsToReplace, solutionForPort);

        return new Replacement(replacementCalculator.getComponentReplacements(),
                replacementCalculator.getConnectorReplacements(),
                replacementCalculator.getPortReplacements());
    }

    private Set<EMAVertex> calculateComponentsToBreakLoops(StrongConnectedComponent strongConnectedComponent) {
        Set<EMAVertex> res = new HashSet<>();
        for (SimpleCycle simpleCycle : strongConnectedComponent.getSimpleCycles()) {
            Set<EMAVertex> outs = simpleCycle.getOutports().stream().map(o -> o.getEmaVertex()).collect(Collectors.toSet());
            res.addAll(outs);
        }
        return res;
    }

    private void applyReplacements(GlobalScope scope, Replacement replacements) {
        for (ComponentReplacement componentReplacement : replacements.getComponentReplacements()) {
            EMAComponentInstanceSymbol parent = scope.<EMAComponentInstanceSymbol>resolve(
                    componentReplacement.getParentComponent(), EMAComponentInstanceSymbol.KIND).orElse(null);
            // Remove old instance
            EMAComponentInstanceSymbol oldChild = parent.getSpannedScope().<EMAComponentInstanceSymbol>resolveLocally(
                    componentReplacement.getOldInstanceName(), EMAComponentInstanceSymbol.KIND).get();
            parent.getSpannedScope().getAsMutableScope().remove(oldChild);

            // Resolve new instance
            String newChildFullName = componentReplacement.getPackageName() + "."
                    + firstCharToLowerCase(componentReplacement.getType());
            EMADynamicComponentInstanceSymbol newChild = scope.<EMADynamicComponentInstanceSymbol>resolve(
                    newChildFullName, EMADynamicComponentInstanceSymbol.KIND).orElse(null);
            // Rename and repackage
            newChild = InstanceCreator.rename(newChild, componentReplacement.getNewInstanceName());
            newChild.setPackageName(parent.getFullName());
            newChild.setFullName(parent.getFullName() + "." + componentReplacement.getNewInstanceName());
            parent.getSpannedScope().getAsMutableScope().add(newChild);
        }
        for (PortReplacement portReplacement : replacements.getPortReplacements()) {
            EMAComponentInstanceSymbol component = scope.<EMAComponentInstanceSymbol>resolve(
                    portReplacement.getComponent(), EMAComponentInstanceSymbol.KIND).orElse(null);

            EMAPortInstanceSymbol newPort = InstanceCreator.createPortInstanceSymbol(portReplacement.getName(),
                    component.getName(), portReplacement.getType(), portReplacement.isIncoming(),
                    component.getSpannedScope().getAsMutableScope());
            component.getSpannedScope().getAsMutableScope().add(newPort);
        }
        for (ConnectorReplacement connectorReplacement : replacements.getConnectorReplacements()) {
            EMAComponentInstanceSymbol component = scope.<EMAComponentInstanceSymbol>resolve(
                    connectorReplacement.getParentComponent(), EMAComponentInstanceSymbol.KIND).orElse(null);

            EMADynamicConnectorInstanceSymbol newConnector = InstanceCreator.createConnectorInstanceSymbol(connectorReplacement.getSource(), connectorReplacement.getTarget(),
                    component.getName(), component.getSpannedScope().getAsMutableScope());
            component.getSpannedScope().getAsMutableScope().add(newConnector);
        }
    }

    private String firstCharToLowerCase(String string) {
        char c[] = string.toCharArray();
        c[0] = Character.toLowerCase(c[0]);
        return new String(c);
    }
}
