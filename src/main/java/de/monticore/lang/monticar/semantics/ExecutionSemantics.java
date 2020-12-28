/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.executionOrder.ExecutionOrder;
import de.monticore.lang.monticar.semantics.loops.detection.EMALoop;
import de.monticore.lang.monticar.semantics.loops.detection.LoopDetection;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StronglyConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.symbols.LoopComponentSymbolInstance;
import de.monticore.lang.monticar.semantics.resolve.Resolver;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class ExecutionSemantics {

    private final TaggingResolver globalScope;
    private final EMAComponentInstanceSymbol rootComponent;

    private boolean resolveLoops = false;
    private boolean handleArtificialLoops = false;
    private boolean solveSymbolicLoops = true;
    private boolean solveSymbolicSpecification = true;

    private boolean warnLoops = true;
    private boolean warnArtificialLoops = true;
    private boolean logSymbolicSolve = true;

    public ExecutionSemantics(TaggingResolver globalScope, EMAComponentInstanceSymbol rootComponent) {
        this.globalScope = globalScope;
        this.rootComponent = rootComponent;
    }

    public void addExecutionSemantics() {
        if (rootComponent == null)
            Log.error("TODO component is null, cannot calculate execution semantics");
        Set<StronglyConnectedComponent> stronglyConnectedComponents = LoopDetection.detectLoops(rootComponent);

        checkViableOptions(stronglyConnectedComponents);

        Resolver resolver = new Resolver(globalScope, rootComponent);
        resolver.resolveLoopSymbols(stronglyConnectedComponents, handleArtificialLoops);

        if (solveSymbolicLoops) {
            Map<LoopComponentSymbolInstance, EMAComponentInstanceSymbol> solvedSymbols
                    = resolver.doSymbolicSolveOfLoopSymbols();
            logSymbolicSolveOfLoops(solvedSymbols);
        }

        if (solveSymbolicSpecification) {
            Map<EMAComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols =
                    resolver.doSymbolicSolveOfSpecifications();
            logSymbolicSolveOfSpecifications(solvedSymbols);
        }

        Set<StronglyConnectedComponent> artificialLoops =
                stronglyConnectedComponents.stream().filter(EMALoop::isArtificial).collect(Collectors.toSet());
        ExecutionOrder.calculateExecutionOrder(rootComponent, artificialLoops, handleArtificialLoops);
    }

    private void logSymbolicSolveOfSpecifications(Map<EMAComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols) {
//        if (logSymbolicSolve)
//            Log.warn("TODO There are analytical solutions to component");
    }

    private void logSymbolicSolveOfLoops(Map<LoopComponentSymbolInstance, EMAComponentInstanceSymbol> solvedSymbols) {
        if (logSymbolicSolve)
            Log.warn("TODO There are analytical solutions to component");
    }

    private void checkViableOptions(Set<StronglyConnectedComponent> stronglyConnectedComponents) {
        if (stronglyConnectedComponents.size() > 0) {
            boolean onlyArtificial = true;
            boolean containsArtificial = false;

            for (StronglyConnectedComponent scc : stronglyConnectedComponents) {
                onlyArtificial &= scc.isArtificial();
                containsArtificial |= scc.isArtificial();
                for (SimpleCycle simpleCycle : scc.getSimpleCycles())
                    containsArtificial |= simpleCycle.isArtificial();
            }

            if (!onlyArtificial && !resolveLoops)
                errorUnresolvedLoops(stronglyConnectedComponents);

            if (!onlyArtificial && resolveLoops)
                warnLoops(stronglyConnectedComponents);

            if (containsArtificial && !handleArtificialLoops)
                warnArtificialLoops(stronglyConnectedComponents);
        }
    }

    private void warnLoops(Set<StronglyConnectedComponent> stronglyConnectedComponents) {
        if (warnLoops)
            Log.warn("TODO There are loops");
    }

    private void warnArtificialLoops(Set<StronglyConnectedComponent> stronglyConnectedComponents) {
        if (warnArtificialLoops)
            Log.warn("TODO There are artificial loops");
    }

    private void errorUnresolvedLoops(Set<StronglyConnectedComponent> stronglyConnectedComponents) {
        Log.error("TODO There are loops");
    }


    public void setResolveLoops(boolean resolveLoops) {
        this.resolveLoops = resolveLoops;
    }

    public void setSolveSymbolicLoops(boolean solveSymbolicLoops) {
        this.solveSymbolicLoops = solveSymbolicLoops;
    }

    public void setHandleArtificialLoops(boolean handleArtificialLoops) {
        this.handleArtificialLoops = handleArtificialLoops;
    }

    public void setWarnLoops(boolean warnLoops) {
        this.warnLoops = warnLoops;
    }

    public void setSolveSymbolicSpecification(boolean solveSymbolicSpecification) {
        this.solveSymbolicSpecification = solveSymbolicSpecification;
    }

    public void setWarnArtificialLoops(boolean warnArtificialLoops) {
        this.warnArtificialLoops = warnArtificialLoops;
    }

    public void setLogSymbolicSolve(boolean logSymbolicSolve) {
        this.logSymbolicSolve = logSymbolicSolve;
    }
}
