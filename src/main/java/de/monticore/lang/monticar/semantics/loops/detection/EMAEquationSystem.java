/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopKind;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class EMAEquationSystem {

    private static int index = 1;
    private static final String EQUATION_SYSTEM_NAME = "EquationSystem";

    private String name;

    private LoopKind kind = LoopKind.Default;

    private Collection<EMAComponentInstanceSymbol> components;
    private Collection<EMAConnectorInstanceSymbol> connectors;

    private Collection<EMAPortInstanceSymbol> inports;
    private Collection<EMAPortInstanceSymbol> outports;

    private Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> atomicSources;
    private Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> atomicTargets;

    private Optional<Map<EMAPortInstanceSymbol, String>> solution = Optional.empty();

    EMAEquationSystem(Collection<EMAComponentInstanceSymbol> components, Collection<EMAConnectorInstanceSymbol> connectors, Collection<EMAPortInstanceSymbol> inports, Collection<EMAPortInstanceSymbol> outports, Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> atomicSources, Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> atomicTargets) {
        this.components = components;
        this.connectors = connectors;
        this.inports = inports;
        this.outports = outports;
        this.atomicSources = atomicSources;
        this.atomicTargets = atomicTargets;
        this.name = EQUATION_SYSTEM_NAME + "_" + index;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Set<EMAComponentInstanceSymbol> getComponentInstanceSymbols() {
        Set<EMAComponentInstanceSymbol> res = new HashSet<>();
        for (EMAComponentInstanceSymbol subcomponent : components)
            if (components instanceof EMAEquationSystem)
                res.addAll(((EMAEquationSystem) components).getComponentInstanceSymbols());
            else
                res.add(subcomponent);

        return res;
    }

    public Optional<EMAPortInstanceSymbol> getAtomicSourceOf(EMAPortInstanceSymbol port) {
        return Optional.ofNullable(atomicSources.getOrDefault(port, null));
    }

    public Set<EMAPortInstanceSymbol> getAtomicTargetsOf(EMAPortInstanceSymbol port) {
        return atomicTargets.getOrDefault(port, new HashSet<>());
    }

    LoopKind getLoopKind() {
        return kind;
    }

    void setLoopKind(LoopKind loopKind) {
        this.kind = loopKind;
    }

    public Collection<EMAConnectorInstanceSymbol> getConnectors() {
        return connectors;
    }

    public Collection<EMAPortInstanceSymbol> getInports() {
        return inports;
    }

    public Collection<EMAPortInstanceSymbol> getOutports() {
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
            Log.error("TODO solution not present");
        return solution.get();
    }

    public String getSolution(EMAPortInstanceSymbol port) {
        if (!isPresentSolution(port))
            Log.error("TODO solution not present");
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

    public Collection<EMAPortInstanceSymbol> getIncomingPortInstances() {
        return getComponentInstanceSymbols()
                .stream()
                .map(c -> c.getIncomingPortInstances())
                .flatMap(Collection::stream)
                .collect(Collectors.toSet());
    }

    public Collection<EMAPortInstanceSymbol> getOutgoingPortInstances() {
        return getComponentInstanceSymbols()
                .stream()
                .map(c -> c.getOutgoingPortInstances())
                .flatMap(Collection::stream)
                .collect(Collectors.toSet());
    }
}
