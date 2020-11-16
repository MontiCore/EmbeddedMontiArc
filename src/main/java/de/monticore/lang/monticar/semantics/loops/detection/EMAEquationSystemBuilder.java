/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnector;
import de.monticore.lang.monticar.semantics.resolve.SymbolTableHelper;
import de.se_rwth.commons.Joiners;

import java.util.*;
import java.util.stream.Collectors;

public final class EMAEquationSystemBuilder {
    Collection<EMAComponentInstanceSymbol> components = new HashSet<>();
    Collection<EMAConnectorInstanceSymbol> connectors = new HashSet<>();
    Collection<EMAPortInstanceSymbol> inports = new HashSet<>();
    Collection<EMAPortInstanceSymbol> outports = new HashSet<>();
    Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> atomicSources = new HashMap<>();
    Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> atomicTargets = new HashMap<>();

    private EMAEquationSystemBuilder() {
    }

    public static EMAEquationSystemBuilder builder() {
        return new EMAEquationSystemBuilder();
    }

    public EMAEquationSystemBuilder setComponents(Collection<EMAComponentInstanceSymbol> components) {
        this.components = components;
        return this;
    }

    public EMAEquationSystemBuilder setConnectors(Collection<EMAConnectorInstanceSymbol> connectors) {
        this.connectors = connectors;
        return this;
    }

    public EMAEquationSystemBuilder setInports(Collection<EMAPortInstanceSymbol> inports) {
        this.inports = inports;
        return this;
    }

    public EMAEquationSystemBuilder setOutports(Collection<EMAPortInstanceSymbol> outports) {
        this.outports = outports;
        return this;
    }

    public EMAEquationSystemBuilder setAtomicSources(Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> atomicSources) {
        this.atomicSources = atomicSources;
        return this;
    }

    public EMAEquationSystemBuilder setAtomicTargets(Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> atomicTargets) {
        this.atomicTargets = atomicTargets;
        return this;
    }

    public EMAEquationSystem build() {
        return new EMAEquationSystem(components, connectors, inports, outports, atomicSources, atomicTargets);
    }

    public static EMAEquationSystem buildFrom(Collection<EMAComponentInstanceSymbol> components) {
        Map<EMAComponentInstanceSymbol, String> nameMapping = new HashMap<>();

        Set<EMAAtomicConnector> atomicConnectors = new HashSet<>();
        for (EMAComponentInstanceSymbol component : components) {
            String newName = formattedNameOf(component);
            nameMapping.put(component, newName);
            component.getIncomingPortInstances()
                    .stream()
                    .forEachOrdered(in -> {
                        Optional<EMAPortInstanceSymbol> source = SymbolTableHelper.getAtomicSourceOf(in);
                        if (source.isPresent())
                            atomicConnectors.add(new EMAAtomicConnector(source.get(), in));
                    });
            component.getOutgoingPortInstances()
                    .stream()
                    .forEachOrdered(out -> atomicConnectors.addAll(
                            SymbolTableHelper.getAtomicTargetsOf(out)
                                    .stream()
                                    .map(target -> new EMAAtomicConnector(out, target))
                                    .collect(Collectors.toSet())
                    ));
        }

        Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> atomicSources = new HashMap<>();
        Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> atomicTargets = new HashMap<>();

        // inner connectors
        Set<EMAConnectorInstanceSymbol> innerConnectors = new HashSet<>();
        atomicConnectors.stream()
                .filter(c -> components.contains(c.getSource().getComponentInstance())
                        && components.contains(c.getTarget().getComponentInstance()))
                .forEachOrdered(c -> {
                    innerConnectors.add(buildConnectorFrom(c, nameMapping));
                    atomicSources.put(c.getTarget(), c.getSource());
                    atomicTargets.getOrDefault(c.getSource(), new HashSet<>()).add(c.getTarget());
                });

        // inports
        Set<EMAPortInstanceSymbol> inportSymbols = new HashSet<>();
        atomicConnectors.stream()
                .filter(c -> !components.contains(c.getSource().getComponentInstance()))
                .forEachOrdered(c -> {
                    inportSymbols.add(c.getTarget());
                    atomicSources.put(c.getTarget(), c.getSource());
                    atomicTargets.getOrDefault(c.getSource(), new HashSet<>()).add(c.getTarget());
                });

        // outports
        Set<EMAPortInstanceSymbol> outportSymbols = new HashSet<>();
        atomicConnectors.stream()
                .filter(c -> !components.contains(c.getTarget().getComponentInstance()))
                .forEachOrdered(c -> {
                    outportSymbols.add(c.getSource());
                    atomicSources.put(c.getTarget(), c.getSource());
                    atomicTargets.getOrDefault(c.getSource(), new HashSet<>()).add(c.getTarget());
                });


        return builder()
                .setComponents(components)
                .setConnectors(innerConnectors)
                .setInports(inportSymbols)
                .setOutports(outportSymbols)
                .setAtomicSources(atomicSources)
                .setAtomicTargets(atomicTargets)
                .build();
    }

    private static EMAConnectorInstanceSymbol buildConnectorFrom(EMAAtomicConnector c, Map<EMAComponentInstanceSymbol, String> nameMapping) {
        String source = formattedQualifiedNameOf(c.getSource(), nameMapping);
        String target =  formattedQualifiedNameOf(c.getTarget(), nameMapping);
        EMAConnectorSymbol connectorSymbol = EMAConnectorInstanceSymbol.builder()
                .setSource(source)
                .setTarget(target)
                .build();
        return EMAConnectorBuilder.instantiate(connectorSymbol, "");
    }


    private static String formattedNameOf(EMAComponentInstanceSymbol component) {
        return NameHelper.replaceWithUnderScore(NameHelper.calculateFullQualifiedNameOf(component));
    }

    private static String formattedQualifiedNameOf(EMAPortInstanceSymbol port, Map<EMAComponentInstanceSymbol, String> nameMapping) {
        String instanceName;
        if (nameMapping.containsKey(port.getComponentInstance()))
            instanceName = nameMapping.get(port.getComponentInstance());
        else {
            instanceName = formattedNameOf(port.getComponentInstance());
            nameMapping.put(port.getComponentInstance(), instanceName);
        }
        return Joiners.DOT.join(instanceName, port.getName());
    }
}
