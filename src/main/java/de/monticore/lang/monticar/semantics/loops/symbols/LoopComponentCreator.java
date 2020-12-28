/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.Constants;
import de.monticore.lang.monticar.semantics.construct.MathComponentGenerator;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnector;
import de.monticore.lang.monticar.semantics.resolve.SymbolTableHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;

import java.util.*;
import java.util.stream.Collectors;

public class LoopComponentCreator {

    public static LoopComponentInstanceSymbol createFrom(TaggingResolver scope, EMAComponentInstanceSymbol parent, String name,
                                                         Collection<EMAComponentInstanceSymbol> components) {
        String packageName = parent.getFullName();
        Map<String, String> inports = new HashMap<>();
        Map<String, String> outports = new HashMap<>();
        Map<String, String> subComponents = new HashMap<>();
        Map<String, String> connectors = new HashMap<>();

        Set<EMAAtomicConnector> atomicConnectors = new HashSet<>();
        for (EMAComponentInstanceSymbol component : components) {
            String newName = formattedNameOf(component);
            subComponents.put(newName, component.getComponentType().getFullName());
            component.getIncomingPortInstances()
                    .stream()
                    .forEachOrdered(in -> atomicConnectors.add(new EMAAtomicConnector(getAtomicSourceOf(in), in)));
            component.getOutgoingPortInstances()
                    .stream()
                    .forEachOrdered(out -> atomicConnectors.addAll(
                            getAtomicTargetsOf(out)
                                    .stream()
                                    .map(target -> new EMAAtomicConnector(out, target))
                                    .collect(Collectors.toSet())
                            ));
        }

        Set<EMAPortInstanceSymbol> inportSymbols = new HashSet<>();
        Set<EMAPortInstanceSymbol> outportSymbols = new HashSet<>();
        atomicConnectors.stream()
                .filter(c -> !components.contains(c.getSourcePort().getComponentInstance()))
                .forEachOrdered(c -> inportSymbols.add(c.getTargetPort()));
        atomicConnectors.stream()
                .filter(c -> !components.contains(c.getTargetPort().getComponentInstance()))
                .forEachOrdered(c -> outportSymbols.add(c.getSourcePort()));
        inportSymbols.stream().forEachOrdered(inport -> {
            String newName = "in_" + NameHelper.replaceWithUnderScore(inport.getFullName());
            inports.put(newName, inport.getTypeReference().getName());
        });
        outportSymbols.stream().forEachOrdered(outport -> {
            String newName = "out_" + NameHelper.replaceWithUnderScore(outport.getFullName());
            outports.put(newName, outport.getTypeReference().getName());
        });

        // connectors
        Set<EMAAtomicConnector> innerConnectors = atomicConnectors
                .stream()
                .filter(c -> components.contains(c.getSourcePort().getComponentInstance())
                          && components.contains(c.getTargetPort().getComponentInstance()))
                .collect(Collectors.toSet());

        innerConnectors.stream()
                .forEachOrdered(c -> {
                    String source = formattedQualifiedNameOf(c.getSourcePort());
                    String target =  formattedQualifiedNameOf(c.getTargetPort());
                    connectors.put(source, target);
                });
        atomicConnectors.stream()
                .filter(c -> !components.contains(c.getSourcePort().getComponentInstance()))
                .forEachOrdered(c -> {
                    String source = formattedNameOf(c.getSourcePort());
                    String target = formattedQualifiedNameOf(c.getTargetPort());
                    connectors.put(source, target);
                });
        atomicConnectors.stream()
                .filter(c -> !components.contains(c.getTargetPort().getComponentInstance()))
                .forEachOrdered(c -> {
                    String source = formattedNameOf(c.getSourcePort());
                    String target = formattedQualifiedNameOf(c.getTargetPort());
                    connectors.put(source, target);
                });

        MathComponentGenerator generator = new MathComponentGenerator();
        generator.generate(StringUtils.capitalize(name), packageName, inports, outports, subComponents, connectors, Constants.synthPath);
        String fullName = NameHelper.toInstanceFullQualifiedName(packageName, name);
        EMAComponentInstanceSymbol emaComponentInstanceSymbol = SymbolTableHelper.resolveInstanceTo(scope, fullName, parent);
        LoopComponentInstanceSymbol res = LoopComponentInstanceSymbol.instantiate(emaComponentInstanceSymbol);
        parent.getSpannedScope().getAsMutableScope().remove(emaComponentInstanceSymbol);
        parent.getSpannedScope().getAsMutableScope().add(res);
        return res;
    }

    private static String formattedNameOf(EMAComponentInstanceSymbol component) {
        return NameHelper.replaceWithUnderScore(component.getFullName());
    }

    private static String formattedNameOf(EMAPortInstanceSymbol port) {
        if (port.isIncoming())
            return "in_" + NameHelper.replaceWithUnderScore(port.getFullName());
        else
            return "out_" + NameHelper.replaceWithUnderScore(port.getFullName());
    }

    private static String formattedQualifiedNameOf(EMAPortInstanceSymbol port) {
        return Joiners.DOT.join(formattedNameOf(port.getComponentInstance()), port.getName());
    }

    private static Collection<EMAPortInstanceSymbol> getAtomicTargetsOf(EMAPortInstanceSymbol port) {
        if (port.isIncoming() && isAtomic(port.getComponentInstance())) return Collections.singletonList(port);
        if (!port.getComponentInstance().getParent().isPresent())
            return Collections.singletonList(port);

        EMAComponentInstanceSymbol next;
        if (port.isIncoming()) // targets another subcomponent
            next = port.getComponentInstance();
        else // targets parent
            next = port.getComponentInstance().getParent().get();

        List<EMAConnectorInstanceSymbol> targets =
                next.getConnectorInstances()
                        .stream()
                        .filter(c -> port.equals(c.getSourcePort()))
                        .collect(Collectors.toList());
        if (targets.size() < 1)
            Log.error("TODO");

        Collection<EMAPortInstanceSymbol> targetPorts = new LinkedList<>();
        targets.stream().forEachOrdered(t -> targetPorts.addAll(getAtomicTargetsOf(t.getTargetPort())));
        return targetPorts;
    }

    private static EMAPortInstanceSymbol getAtomicSourceOf(EMAPortInstanceSymbol port) {
        if (port.isOutgoing() && isAtomic(port.getComponentInstance())) return port;
        if (!port.getComponentInstance().getParent().isPresent())
            return port;

        EMAComponentInstanceSymbol next;
        if (port.isOutgoing()) // comes from another subcomponent
            next = port.getComponentInstance();
        else // comes from parent
            next = port.getComponentInstance().getParent().get();

        List<EMAConnectorInstanceSymbol> sources =
                next.getConnectorInstances()
                        .stream()
                        .filter(c -> port.equals(c.getTargetPort()))
                        .collect(Collectors.toList());
        if (sources.size() != 1)
            Log.error("TODO");

        return getAtomicSourceOf(sources.get(0).getSourcePort());
    }

    private static boolean isAtomic(EMAComponentInstanceSymbol componentInstance) {
        return componentInstance.getSubComponents().isEmpty();
    }

}
