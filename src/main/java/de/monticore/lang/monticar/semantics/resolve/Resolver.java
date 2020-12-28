/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSpecificationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.semantics.construct.ReplacementCalculator;
import de.monticore.lang.monticar.semantics.loops.analyze.SpecificationConverter;
import de.monticore.lang.monticar.semantics.loops.detection.LoopDetection;
import de.monticore.lang.monticar.semantics.loops.detection.StronglyConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.symbols.*;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class Resolver {

    private EMAComponentInstanceSymbol rootComponent;
    private TaggingResolver scope;
    private Set<LoopComponentSymbolInstance> loopSymbols = new HashSet<>();

    public Resolver(TaggingResolver scope, EMAComponentInstanceSymbol rootComponent) {
        this.scope = scope;
        this.rootComponent = rootComponent;
    }

    public Resolver(TaggingResolver scope, String rootModel) {
        this.scope = scope;
        this.rootComponent = scope.<EMAComponentInstanceSymbol>resolve(rootModel, EMAComponentInstanceSymbol.KIND).orElse(null);
    }

    public void handleScope() {
        Set<StronglyConnectedComponent> stronglyConnectedComponents = LoopDetection.detectLoops(rootComponent);
        resolveLoopSymbols(stronglyConnectedComponents, true);
        doSymbolicSolveOfLoopSymbols();
        doSymbolicSolveOfSpecifications();
    }

    public Set<LoopComponentSymbolInstance> resolveLoopSymbols(Set<StronglyConnectedComponent> stronglyConnectedComponents,
                                                               boolean handleArtificialLoops) {
        Set<LoopComponentSymbolInstance> loopSymbols = new HashSet<>();

        for (StronglyConnectedComponent stronglyConnectedComponent : stronglyConnectedComponents) {
            if (stronglyConnectedComponent.isArtificial() && handleArtificialLoops)
                continue;
            Set<EMAComponentInstanceSymbol> components = stronglyConnectedComponent.getAllComponents()
                    .stream().collect(Collectors.toSet());

            EMAEquationSystem equationSystem = EMAEquationSystemBuilder.buildFrom(components);
            Set<EMAComponentInstanceSymbol> componentsToBuildLoopSymbolFrom =
                    components.stream().filter(c -> isAKepper(c, equationSystem)).collect(Collectors.toSet());
            Set<EMAComponentInstanceSymbol> componentsToRemove =
                    components.stream().filter(c -> !componentsToBuildLoopSymbolFrom.contains(c)).collect(Collectors.toSet());

            componentsToRemove.stream().forEach(component -> {
                SymbolTableHelper.removeComponentRecursive(component);
            });

            componentsToBuildLoopSymbolFrom.stream().forEach(component -> {
                LoopComponentSymbolInstance loopSymbol = LoopComponentSymbolInstance.instantiate(component, equationSystem);
                SymbolTableHelper.replaceComponent(component, loopSymbol);
                loopSymbol.connectInformation();
                loopSymbols.add(loopSymbol);
            });
        }
        this.loopSymbols = loopSymbols;
        return loopSymbols;
    }

    private boolean isAKepper(EMAComponentInstanceSymbol component, EMAEquationSystem equationSystem) {
        if (equationSystem.getOutports()
                .stream()
                .map(port -> port.getComponentInstance())
                .collect(Collectors.toSet()).contains(component))
            return true;

        // TODO check tags or other attributes
        return false;
    }

    public Map<LoopComponentSymbolInstance, EMAComponentInstanceSymbol> doSymbolicSolveOfLoopSymbols() {
        Map<LoopComponentSymbolInstance, EMAComponentInstanceSymbol> solvedSymbols = new HashMap<>();
        for (LoopComponentSymbolInstance loopSymbol : loopSymbols) {
            if (EMAEquationSystemHelper.trySymbolicSolve(loopSymbol.getEquationSystem())) {
                EMAComponentInstanceSymbol symbolSolution =
                        ReplacementCalculator.generateAndReplaceComponent(scope, loopSymbol,
                        loopSymbol.getEquationSystem().getSolution());
                solvedSymbols.put(loopSymbol, symbolSolution);
            }
        }
        loopSymbols.removeAll(solvedSymbols.keySet());
        return solvedSymbols;
    }

    public Map<EMAComponentInstanceSymbol, EMAComponentInstanceSymbol> doSymbolicSolveOfSpecifications() {
        Map<EMAComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols = new HashMap<>();
        for (EMAComponentInstanceSymbol component : searchForSpecifications(rootComponent)) {
            EMAMSpecificationSymbol specification = SpecificationConverter.convert(component).orElse(null);
            if (specification == null) Log.error("TODO");
            Optional<Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol>> solution =
                    EquationSystemSymbolicSolver.trySymbolicSolve(specification);
            if (solution.isPresent()) {
                Map<EMAPortInstanceSymbol, String> solutionMap =
                        EMAEquationSystemHelper.convertSolutionMap(solution.get(), component.getPortInstanceList());
                EMAComponentInstanceSymbol symbolSolution =
                        ReplacementCalculator.generateAndReplaceComponent(scope, component, solutionMap);
                solvedSymbols.put(component, symbolSolution);
            }
        }
        return solvedSymbols;
    }

    private Collection<EMAComponentInstanceSymbol> searchForSpecifications(EMAComponentInstanceSymbol component) {
        Set<EMAComponentInstanceSymbol> result = new HashSet<>();


        if (isSpecification(component))
            result.add(component);
        else
            for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
                result.addAll(searchForSpecifications(subComponent));

        return result;
    }

    public static boolean isSpecification(EMAComponentInstanceSymbol component) {
        Collection<MathStatementsSymbol> symbols = component.getSpannedScope().resolveLocally(MathStatementsSymbol.KIND);
        if (symbols.size() != 1) return false;
        MathStatementsSymbol mathStatementsSymbol = symbols.stream().findFirst().get();
        Optional<MathExpressionSymbol> first = mathStatementsSymbol.getMathExpressionSymbols()
                .stream()
                .filter(e -> e instanceof EMAMSpecificationSymbol)
                .findFirst();

        if (first.isPresent()) return true;
        else return false;
    }
}
