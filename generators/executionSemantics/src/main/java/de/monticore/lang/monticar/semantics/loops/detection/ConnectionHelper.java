/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper;
import de.se_rwth.commons.Names;

import java.util.*;
import java.util.stream.Collectors;

public class ConnectionHelper {

    private static Map<EMAPortInstanceSymbol, Optional<EMAPortInstanceSymbol>> sourcesCache = new HashMap<>();
    private static Map<EMAPortInstanceSymbol, Collection<EMAPortInstanceSymbol>> targetsCache = new HashMap<>();

    private static Map<EMAComponentInstanceSymbol, Collection<EMAConnectorInstanceSymbol>> connectorsCache = new HashMap<>();

    public static void resetCache() {
        sourcesCache = new HashMap<>();
        targetsCache = new HashMap<>();
        connectorsCache = new HashMap<>();
    }

    public static Collection<EMAPortInstanceSymbol> targetsOf(EMAPortInstanceSymbol port) {
        return targetsOf(port, true);
    }

    public static Collection<EMAPortInstanceSymbol> targetsOf(EMAPortInstanceSymbol port, boolean considerNonVirtual) {
        return targetsOf(port, considerNonVirtual, true);
    }

    public static Collection<EMAPortInstanceSymbol> targetsOf(EMAPortInstanceSymbol port, boolean considerNonVirtual,
                                                              boolean firstCall) {
        if (port == null) return new HashSet<>();
        if (targetsCache.containsKey(port)) return targetsCache.get(port);
        EMAComponentInstanceSymbol portComponentInstance = port.getComponentInstance();
        if (port.isIncoming() && EMAPropertiesHelper.isAtomic(portComponentInstance))
            return Collections.singletonList(port);
        if (port.isOutgoing() && !portComponentInstance.getParent().isPresent())
            return Collections.singletonList(port);
        if (!firstCall &&
                (!portComponentInstance.getParent().isPresent()
                        || considerNonVirtual && EMAPropertiesHelper.isNonVirtual(portComponentInstance)))
            return Collections.singletonList(port);

        EMAComponentInstanceSymbol next;
        String portName;
        if (port.isIncoming()) { // targets another subcomponent
            next = portComponentInstance;
            portName = port.getName();
        } else { // targets parent
            next = portComponentInstance.getParent().get();
            portName = Names.getQualifiedName(portComponentInstance.getName(), port.getName());
        }

        List<EMAConnectorInstanceSymbol> targets =
                getConnectors(next)
                        .stream()
                        .filter(c -> portName.equals(c.getSource()))
                        .collect(Collectors.toList());
//        if (targets.size() < 1)
//            Log.warn(String.format("TODO There is no outgoing connection for port \"%s\"", port.getFullName()));

        Collection<EMAPortInstanceSymbol> targetPorts = new LinkedList<>();
        targets.stream().forEachOrdered(t -> targetPorts.addAll(targetsOf(t.getTargetPort(),
                considerNonVirtual, false)));
        return addToTargetsCashe(port, targetPorts);
    }

    private static Collection<EMAPortInstanceSymbol> addToTargetsCashe(EMAPortInstanceSymbol port,
                                                                       Collection<EMAPortInstanceSymbol> targets) {
        targetsCache.put(port, targets);
        return targets;
    }

    public static Optional<EMAPortInstanceSymbol> sourceOf(EMAPortInstanceSymbol port) {
        return sourceOf(port, true);
    }


    public static Optional<EMAPortInstanceSymbol> sourceOf(EMAPortInstanceSymbol port, boolean considerNonVirtual) {
        return sourceOf(port, considerNonVirtual, true);
    }

    private static Optional<EMAPortInstanceSymbol> sourceOf(EMAPortInstanceSymbol port, boolean considerNonVirtual,
                                                            boolean firstCall) {
        if (port == null) return Optional.empty();
        if (sourcesCache.containsKey(port)) return sourcesCache.get(port);
        if (port.isConstant())
            return Optional.of(port);
        EMAComponentInstanceSymbol portComponentInstance = port.getComponentInstance();
        if (port.isOutgoing() && EMAPropertiesHelper.isAtomic(portComponentInstance))
            return Optional.of(port);
        if (port.isIncoming() && !portComponentInstance.getParent().isPresent())
            return Optional.of(port);
        if (!firstCall &&
                (!portComponentInstance.getParent().isPresent()
                        || considerNonVirtual && EMAPropertiesHelper.isNonVirtual(portComponentInstance)))
            return Optional.of(port);

        EMAComponentInstanceSymbol next;
        String portName;
        if (port.isOutgoing()) { // comes from another subcomponent
            next = portComponentInstance;
            portName = port.getName();
        } else { // comes from parent
            next = portComponentInstance.getParent().get();
            portName = Names.getQualifiedName(portComponentInstance.getName(), port.getName());
        }

        List<EMAConnectorInstanceSymbol> sources =
                getConnectors(next)
                        .stream()
                        .filter(c -> portName.equals(c.getTarget()))
                        .collect(Collectors.toList());
        if (sources.size() != 1) {
            if (port.isConfig())
                return Optional.of(port);
            else {
//                Log.warn(String.format("TODO There is no incoming connection for port \"%s\"", port.getFullName()));
                return Optional.empty();
            }
        }

        return addToSourcesCashe(port, sourceOf(sources.get(0).getSourcePort(), considerNonVirtual, false));
    }

    private static Optional<EMAPortInstanceSymbol> addToSourcesCashe(EMAPortInstanceSymbol port,
                                                                     Optional<EMAPortInstanceSymbol> source) {
        sourcesCache.put(port, source);
        return source;
    }

    private static Collection<EMAConnectorInstanceSymbol> getConnectors(EMAComponentInstanceSymbol component) {
        return connectorsCache.getOrDefault(component, component.getConnectorInstances());
    }

}
