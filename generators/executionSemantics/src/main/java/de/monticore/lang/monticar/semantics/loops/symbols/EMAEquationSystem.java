/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.analyze.EquationSystemType;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnectorInstance;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class EMAEquationSystem {

    private static int index = 1;
    private static final String EQUATION_SYSTEM_NAME = "EquationSystem";
    private final CommonScope spannedScope;

    private String name;

    private EquationSystemType type = EquationSystemType.Constant;

    private Collection<EMAComponentInstanceSymbol> components;
    private Collection<EMAAtomicConnectorInstance> connectors;

    private Collection<EMAPortInstanceSymbol> inports;
    private Collection<EMAPortInstanceSymbol> outports;

    private Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> atomicSources;
    private Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> atomicTargets;

    private Optional<Map<EMAPortInstanceSymbol, String>> solution = Optional.empty();

    EMAEquationSystem(Collection<EMAComponentInstanceSymbol> components, Collection<EMAAtomicConnectorInstance> connectors, Collection<EMAPortInstanceSymbol> inports, Collection<EMAPortInstanceSymbol> outports, Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> atomicSources, Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> atomicTargets) {
        this.connectors = connectors;
        this.inports = inports;
        this.outports = outports;
        this.atomicSources = atomicSources;
        this.atomicTargets = atomicTargets;
        this.name = EQUATION_SYSTEM_NAME + "_" + index;
        this.spannedScope = new CommonScope();
        Optional<EMAComponentInstanceSymbol> any = components.stream().findAny();
        if (any.isPresent()) {
            Scope enclosingScope = any.get().getEnclosingScope();
            if (enclosingScope != null) {
                spannedScope.setResolvingFilters(enclosingScope.getResolvingFilters());
            }
        }
        this.spannedScope.setName(getName());
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
        this.spannedScope.setName(name);
    }

    public Set<EMAComponentInstanceSymbol> getComponentInstanceSymbols() {
        Map<String, Collection<Symbol>> localSymbols = spannedScope.getLocalSymbols();
        Set<EMAComponentInstanceSymbol> result = new HashSet<>();
        for (Collection<Symbol> symbols : localSymbols.values()) {
            for (Symbol symbol : symbols) {
                if (symbol.isKindOf(EMAComponentInstanceSymbol.KIND)) {
                    result.add((EMAComponentInstanceSymbol) symbol);
                }
            }
        }
        return result;
    }

    public Optional<EMAPortInstanceSymbol> getAtomicSourceOf(EMAPortInstanceSymbol port) {
        return Optional.ofNullable(atomicSources.getOrDefault(port, null));
    }

    public Set<EMAPortInstanceSymbol> getAtomicTargetsOf(EMAPortInstanceSymbol port) {
        return atomicTargets.getOrDefault(port, new HashSet<>());
    }

    EquationSystemType getType() {
        return type;
    }

    void setType(EquationSystemType equationSystemType) {
        this.type = equationSystemType;
    }

    public Collection<EMAAtomicConnectorInstance> getConnectors() {
        return connectors;
    }

    public Collection<EMAPortInstanceSymbol> getIncomingPorts() {
        return inports;
    }

    public Collection<EMAPortInstanceSymbol> getOutgoingPorts() {
        return outports;
    }

    public Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> getAtomicSources() {
        return atomicSources;
    }

    public Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> getAtomicTargets() {
        return atomicTargets;
    }

    public Map<EMAPortInstanceSymbol, String> getSolution() {
        if (!isPresentSolution())
            Log.error("0xEMAES3911 solution not present");
        return solution.get();
    }

    public String getSolution(EMAPortInstanceSymbol port) {
        if (!isPresentSolution(port))
            Log.error("0xEMAES3912 solution not present");
        return solution.get().get(port);
    }

    public void setSolution(Map<EMAPortInstanceSymbol, String> solution) {
        this.solution = Optional.ofNullable(solution);
    }

    public void setSolution(EMAPortInstanceSymbol port, String solution) {
        this.solution.orElse(new HashMap<>()).put(port, solution);
    }

    public boolean isPresentSolution() {
        return solution.isPresent();
    }

    public boolean isPresentSolution(EMAPortInstanceSymbol port) {
        return solution.isPresent() && getSolution().containsKey(port);
    }

    public Collection<EMAPortInstanceSymbol> getComponentIncomingPortInstances() {
        return getComponentInstanceSymbols()
                .stream()
                .map(c -> c.getIncomingPortInstances())
                .flatMap(Collection::stream)
                .collect(Collectors.toSet());
    }

    public Collection<EMAPortInstanceSymbol> getComponentOutgoingPortInstances() {
        return getComponentInstanceSymbols()
                .stream()
                .map(c -> c.getOutgoingPortInstances())
                .flatMap(Collection::stream)
                .collect(Collectors.toSet());
    }

    public void addComponentsToLocalScope(Set<EMAComponentInstanceSymbol> components) {
        components.stream().forEach(c -> spannedScope.add(c));
    }
}
