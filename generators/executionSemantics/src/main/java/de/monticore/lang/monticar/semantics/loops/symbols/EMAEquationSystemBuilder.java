/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnectorInstance;
import de.se_rwth.commons.Joiners;

import java.util.*;
import java.util.stream.Collectors;

public final class EMAEquationSystemBuilder {
    Collection<EMAComponentInstanceSymbol> components = new HashSet<>();
    Collection<EMAAtomicConnectorInstance> connectors = new HashSet<>();
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

    public EMAEquationSystemBuilder setConnectors(Collection<EMAAtomicConnectorInstance> connectors) {
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

        Set<EMAAtomicConnectorInstance> atomicConnectors = new HashSet<>();
        for (EMAComponentInstanceSymbol component : components) {
            String newName = formattedNameOf(component);
            nameMapping.put(component, newName);
            component.getIncomingPortInstances()
                    .stream()
                    .forEachOrdered(in -> {
                        Optional<EMAPortInstanceSymbol> source = ConnectionHelper.sourceOf(in);
                        if (source.isPresent())
                            atomicConnectors.add(new EMAAtomicConnectorInstance(source.get(), in));
                    });
            component.getOutgoingPortInstances()
                    .stream()
                    .forEachOrdered(out -> atomicConnectors.addAll(
                            ConnectionHelper.targetsOf(out)
                                    .stream()
                                    .map(target -> new EMAAtomicConnectorInstance(out, target))
                                    .collect(Collectors.toSet())
                    ));
        }

        Map<EMAPortInstanceSymbol, EMAPortInstanceSymbol> atomicSources = new HashMap<>();
        Map<EMAPortInstanceSymbol, Set<EMAPortInstanceSymbol>> atomicTargets = new HashMap<>();

        // inner connectors
        atomicConnectors.stream()
                .filter(c -> components.contains(c.getSourcePort().getComponentInstance())
                        && components.contains(c.getTargetPort().getComponentInstance()))
                .forEachOrdered(c -> {
                    atomicSources.put(c.getTargetPort(), c.getSourcePort());
                    atomicTargets.getOrDefault(c.getSourcePort(), new HashSet<>()).add(c.getTargetPort());
                });

        // inports
        Set<EMAPortInstanceSymbol> inportSymbols = new HashSet<>();
        atomicConnectors.stream()
                .filter(c -> !components.contains(c.getSourcePort().getComponentInstance()))
                .forEachOrdered(c -> {
                    inportSymbols.add(c.getTargetPort());
                    atomicSources.put(c.getTargetPort(), c.getSourcePort());
                    atomicTargets.getOrDefault(c.getSourcePort(), new HashSet<>()).add(c.getTargetPort());
                });

        // outports
        Set<EMAPortInstanceSymbol> outportSymbols = new HashSet<>();
        atomicConnectors.stream()
                .filter(c -> !components.contains(c.getTargetPort().getComponentInstance()))
                .forEachOrdered(c -> {
                    outportSymbols.add(c.getSourcePort());
                    atomicSources.put(c.getTargetPort(), c.getSourcePort());
                    atomicTargets.getOrDefault(c.getSourcePort(), new HashSet<>()).add(c.getTargetPort());
                });


        return builder()
                .setComponents(components)
                .setConnectors(atomicConnectors)
                .setInports(inportSymbols)
                .setOutports(outportSymbols)
                .setAtomicSources(atomicSources)
                .setAtomicTargets(atomicTargets)
                .build();
    }

    private static EMAConnectorInstanceSymbol buildConnectorFrom(EMAAtomicConnectorInstance c,
                                                                 Map<EMAComponentInstanceSymbol, String> nameMapping) {
        String source = formattedQualifiedNameOf(c.getSourcePort(), nameMapping);
        String target =  formattedQualifiedNameOf(c.getTargetPort(), nameMapping);
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
