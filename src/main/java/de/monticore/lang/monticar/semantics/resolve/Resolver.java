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
import de.monticore.lang.monticar.semantics.loops.detection.*;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class Resolver {

    private EMAComponentInstanceSymbol rootComponent;
    private TaggingResolver scope;
    private Set<LoopSymbolInstance> loopSymbols = new HashSet<>();

    public Resolver(TaggingResolver scope, String rootModel) {
        Log.init();
        Log.enableFailQuick(true);
        this.scope = scope;
        this.rootComponent = scope.<EMAComponentInstanceSymbol>resolve(rootModel, EMAComponentInstanceSymbol.KIND).orElse(null);
    }

    public void handleScope() {
        resolveLoopSymbols();
        doSymbolicSolveOfLoopSymbols();
        doSymbolicSolveOfSpecifications();
    }

    public Set<LoopSymbolInstance> resolveLoopSymbols() {
        Set<StrongConnectedComponent> strongConnectedComponents = Detection.detectLoops(rootComponent);
        Set<LoopSymbolInstance> loopSymbols = new HashSet<>();

        for (StrongConnectedComponent strongConnectedComponent : strongConnectedComponents) {
            Set<EMAComponentInstanceSymbol> components = strongConnectedComponent.getAllComponents()
                    .stream().map(c -> c.getReferencedSymbol()).collect(Collectors.toSet());

            EMAEquationSystem equationSystem = EMAEquationSystemBuilder.buildFrom(components);
            Set<EMAComponentInstanceSymbol> componentsToBuildLoopSymbolFrom =
                    components.stream().filter(c -> isAKepper(c, equationSystem)).collect(Collectors.toSet());
            Set<EMAComponentInstanceSymbol> componentsToRemove =
                    components.stream().filter(c -> !componentsToBuildLoopSymbolFrom.contains(c)).collect(Collectors.toSet());

            componentsToRemove.stream().forEach(component -> {
                SymbolTableHelper.removeComponentRecursive(component);
            });

            componentsToBuildLoopSymbolFrom.stream().forEach(component -> {
                LoopSymbolInstance loopSymbol = LoopSymbolInstance.instantiate(component, equationSystem);
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

    public void doSymbolicSolveOfLoopSymbols() {
        Set<LoopSymbolInstance> solvedSymbols = new HashSet<>();
        for (LoopSymbolInstance loopSymbol : loopSymbols) {
            if (EMAEquationSystemHelper.trySymbolicSolve(loopSymbol.getEquationSystem())) {
                ReplacementCalculator.generateAndReplaceComponent(scope, loopSymbol,
                        loopSymbol.getEquationSystem().getSolution());
                solvedSymbols.add(loopSymbol);
            }
        }
        loopSymbols.removeAll(solvedSymbols);
    }

    public void doSymbolicSolveOfSpecifications() {
        for (EMAComponentInstanceSymbol component : searchForSpecifications(rootComponent)) {
            EMAMSpecificationSymbol specification = SpecificationConverter.convert(component).orElse(null);
            if (specification == null) Log.error("TODO");
            Optional<Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol>> solution = EquationSystemSymbolicSolver.trySymbolicSolve(specification);
            if (solution.isPresent()) {
                Map<EMAPortInstanceSymbol, String> solutionMap = EMAEquationSystemHelper.convertSolutionMap(solution.get(), component.getPortInstanceList());
                ReplacementCalculator.generateAndReplaceComponent(scope, component, solutionMap);
            }
        }

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
