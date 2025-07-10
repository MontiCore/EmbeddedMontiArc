/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.executionOrder.ExecutionOrder;
import de.monticore.lang.monticar.semantics.loops.detection.*;
import de.monticore.lang.monticar.semantics.loops.symbols.LoopComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.resolve.Resolver;
import de.monticore.lang.monticar.semantics.util.BasicLibrary;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.prettyprint.IndentPrinter;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class ExecutionSemantics {

    private final TaggingResolver globalScope;
    private final EMAComponentInstanceSymbol rootComponent;

    public static boolean RESOLVE_LOOPS = false;
    public static boolean HANDLE_ARTIFICIAL_LOOPS = false;
    public static boolean SOLVE_LOOPS_SYMBOLIC = true;
    public static boolean SOLVE_SPECIFICATIONS_SYMBOLIC = true;

    public static boolean WARN_LOOPS = true;
    public static boolean WARN_ARTIFICIAL_LOOPS = true;
    public static boolean LOG_SYMBOLIC_SOLVE = true;

    public ExecutionSemantics(TaggingResolver globalScope, EMAComponentInstanceSymbol rootComponent) {
        BasicLibrary.extract();
        this.globalScope = globalScope;
        this.rootComponent = rootComponent;
    }

    public void addExecutionSemantics() {
        if (rootComponent == null)
            Log.error("0xEMAES0001  component is null, cannot calculate execution semantics");
        Set<StronglyConnectedComponent> stronglyConnectedComponents = LoopDetection.detectLoops(rootComponent);
        checkViableOptions(stronglyConnectedComponents);

        Resolver resolver = new Resolver(globalScope, rootComponent);
        if (SOLVE_SPECIFICATIONS_SYMBOLIC) {
            Map<EMAComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols =
                    resolver.doSymbolicSolveOfSpecifications();
            logSymbolicSolveOfSpecifications(solvedSymbols);
            stronglyConnectedComponents = LoopDetection.detectLoops(rootComponent);
        }

        resolver.resolveLoopSymbols(stronglyConnectedComponents, HANDLE_ARTIFICIAL_LOOPS);

        if (SOLVE_LOOPS_SYMBOLIC) {
            Map<LoopComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols
                    = resolver.doSymbolicSolveOfLoopSymbols();
            logSymbolicSolveOfLoops(solvedSymbols);
        }

        Set<StronglyConnectedComponent> artificialLoops =
                stronglyConnectedComponents.stream().filter(EMALoop::isArtificial).collect(Collectors.toSet());
        ExecutionOrder.calculateExecutionOrder(rootComponent, artificialLoops, HANDLE_ARTIFICIAL_LOOPS);
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
        if (LOG_SYMBOLIC_SOLVE) {
            for (EMAComponentInstanceSymbol solved : solvedSymbols.keySet()) {
                Log.warn(String.format("Solved component \"%s\" analytically", solved.getFullName()));
            }
        }
    }

    private void logSymbolicSolveOfLoops(Map<LoopComponentInstanceSymbol, EMAComponentInstanceSymbol> solvedSymbols) {
        if (LOG_SYMBOLIC_SOLVE) {
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

            if (!onlyArtificial && !RESOLVE_LOOPS)
                errorUnresolvedLoops(stronglyConnectedComponents);

            if (!onlyArtificial && RESOLVE_LOOPS)
                warnLoops(stronglyConnectedComponents);

            if (containsArtificial && !HANDLE_ARTIFICIAL_LOOPS)
                warnArtificialLoops(stronglyConnectedComponents);
        }
    }

    private void warnLoops(Set<StronglyConnectedComponent> stronglyConnectedComponents) {
        if (WARN_LOOPS) {
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
        if (WARN_ARTIFICIAL_LOOPS) {
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
        this.RESOLVE_LOOPS = resolveLoops;
    }

    public void setSolveSymbolicLoops(boolean solveSymbolicLoops) {
        this.SOLVE_LOOPS_SYMBOLIC = solveSymbolicLoops;
    }

    public void setHandleArtificialLoops(boolean handleArtificialLoops) {
        this.HANDLE_ARTIFICIAL_LOOPS = handleArtificialLoops;
    }

    public void setWarnLoops(boolean warnLoops) {
        this.WARN_LOOPS = warnLoops;
    }

    public void setSolveSymbolicSpecification(boolean solveSymbolicSpecification) {
        this.SOLVE_SPECIFICATIONS_SYMBOLIC = solveSymbolicSpecification;
    }

    public void setWarnArtificialLoops(boolean warnArtificialLoops) {
        this.WARN_ARTIFICIAL_LOOPS = warnArtificialLoops;
    }

    public void setLogSymbolicSolve(boolean logSymbolicSolve) {
        this.LOG_SYMBOLIC_SOLVE = logSymbolicSolve;
    }
}
