/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.executionOrder.ExecutionOrder;
import de.monticore.lang.monticar.semantics.helper.Find;
import de.monticore.lang.monticar.semantics.loops.detection.EMALoop;
import de.monticore.lang.monticar.semantics.loops.detection.LoopDetection;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StronglyConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.symbols.LoopComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.resolve.Resolver;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.prettyprint.IndentPrinter;
import de.se_rwth.commons.logging.Log;

import java.util.*;
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
            Map<LoopComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols
                    = resolver.doSymbolicSolveOfLoopSymbols();
            logSymbolicSolveOfLoops(solvedSymbols);
        }

        if (solveSymbolicSpecification) {
            Map<EMAComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols =
                    resolver.doSymbolicSolveOfSpecifications();
            logSymbolicSolveOfSpecifications(solvedSymbols);
        }

//        Map<EMAComponentInstanceSymbol, Boolean> virtual = makeAllNonVirtual(Find.allComponents(rootComponent));
//        stronglyConnectedComponents = LoopDetection.detectLoops(rootComponent);

        Set<StronglyConnectedComponent> artificialLoops =
                stronglyConnectedComponents.stream().filter(EMALoop::isArtificial).collect(Collectors.toSet());
        ExecutionOrder.calculateExecutionOrder(rootComponent, artificialLoops, handleArtificialLoops);

//        resetNonVirtual(virtual);
    }

    private void resetNonVirtual(Map<EMAComponentInstanceSymbol, Boolean> virtual) {
        for (EMAComponentInstanceSymbol component : virtual.keySet()) {
            if (virtual.get(component)) {
                Iterator<ASTComponentModifier> modifierIterator = component.getComponentModifiers().iterator();
                while(modifierIterator.hasNext()) {
                    ASTComponentModifier modifier = modifierIterator.next();
                    if (modifier instanceof ASTVirtModifier &&
                            ((ASTVirtModifier) modifier).getVIRTUAL().equals(ASTVIRTUAL.VIRTUAL))
                        modifierIterator.remove();
                }
            }
        }
    }

    private static Map<EMAComponentInstanceSymbol, Boolean> makeAllNonVirtual(Collection<EMAComponentInstanceSymbol> components) {
        Map<EMAComponentInstanceSymbol, Boolean> virtual = new HashMap<>();
        ASTVirtModifier virtModifier =
                EmbeddedMontiArcMill.virtModifierBuilder().setVIRTUAL(ASTVIRTUAL.NONVIRTUAL).build();
        components.stream().forEachOrdered(c -> {
            virtual.put(c, c.isVirtual());
            if (c.isVirtual()) {
                c.getComponentModifiers().add(virtModifier);
            }
        });

        return virtual;
    }

    private void logSymbolicSolveOfSpecifications(Map<EMAComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols) {
        if (logSymbolicSolve) {
            for (EMAComponentInstanceSymbol solved : solvedSymbols.keySet()) {
                Log.warn(String.format("Solved component \"%s\" analytically", solved.getFullName()));
            }
        }
    }

    private void logSymbolicSolveOfLoops(Map<LoopComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols) {
        if (logSymbolicSolve) {
            for (LoopComponentInstanceSymbol solved : solvedSymbols.keySet()) {
                Log.warn(String.format("Solved component \"%s\" analytically", solved.getFullName()));
            }
        }
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
        if (warnLoops) {
            IndentPrinter printer = new IndentPrinter();
            printer.println("Found Loops:");
            printer.indent();
            for (StronglyConnectedComponent loop : stronglyConnectedComponents) {
                printer.println(String.format("Loop consisting of %s components", loop.getAllComponents().size()));
                printer.indent();
                for (EMAComponentInstanceSymbol component : loop.getAllComponents()) {
                    printer.println(String.format("- %s", component.getFullName()));
                }
                printer.unindent();
            }
            printer.unindent();
            Log.warn(printer.getContent());
        }
    }

    private void warnArtificialLoops(Set<StronglyConnectedComponent> stronglyConnectedComponents) {
        if (warnArtificialLoops) {
            IndentPrinter printer = new IndentPrinter();
            printer.println("Found Artificial Loops:");
            printer.indent();
            for (StronglyConnectedComponent loop : stronglyConnectedComponents) {
                printer.println(String.format("Loop consisting of %s components", loop.getAllComponents().size()));
                printer.indent();
                for (EMAComponentInstanceSymbol component : loop.getAllComponents()) {
                    printer.println(String.format("- %s", component.getFullName()));
                }
                printer.unindent();
            }
            printer.unindent();
            Log.warn(printer.getContent());
        }
    }

    private void errorUnresolvedLoops(Set<StronglyConnectedComponent> stronglyConnectedComponents) {
        IndentPrinter printer = new IndentPrinter();
        printer.println("Found Unresolved Loops:");
        printer.indent();
        for (StronglyConnectedComponent loop : stronglyConnectedComponents) {
            printer.println(String.format("Loop consisting of %s components", loop.getAllComponents().size()));
            printer.indent();
            for (EMAComponentInstanceSymbol component : loop.getAllComponents()) {
                printer.println(String.format("- %s", component.getFullName()));
            }
            printer.unindent();
        }
        printer.unindent();
        Log.error(printer.getContent());
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
