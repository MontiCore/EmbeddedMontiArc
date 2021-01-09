/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class ConnectionHelper {

    public static Collection<EMAPortInstanceSymbol> targetsOf(EMAPortInstanceSymbol port) {
        return targetsOf(port, true);
    }

    public static Collection<EMAPortInstanceSymbol> targetsOf(EMAPortInstanceSymbol port, boolean considerNonVirtual) {
        return targetsOf(port, considerNonVirtual, true);
    }

    public static Collection<EMAPortInstanceSymbol> targetsOf(EMAPortInstanceSymbol port, boolean considerNonVirtual,
                                                              boolean firstCall) {
        if (port.isIncoming() && isAtomic(port.getComponentInstance()))
            return Collections.singletonList(port);
        if (port.isOutgoing() && !port.getComponentInstance().getParent().isPresent())
            return Collections.singletonList(port);
        if (!firstCall &&
                (!port.getComponentInstance().getParent().isPresent()
                        || considerNonVirtual && isNonVirtual(port.getComponentInstance())))
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
//        if (targets.size() < 1)
//            Log.warn(String.format("TODO There is no outgoing connection for port \"%s\"", port.getFullName()));

        Collection<EMAPortInstanceSymbol> targetPorts = new LinkedList<>();
        targets.stream().forEachOrdered(t -> targetPorts.addAll(targetsOf(t.getTargetPort(),
                considerNonVirtual, false)));
        return targetPorts;
    }

    public static Optional<EMAPortInstanceSymbol> sourceOf(EMAPortInstanceSymbol port) {
        return sourceOf(port, true);
    }


    public static Optional<EMAPortInstanceSymbol> sourceOf(EMAPortInstanceSymbol port, boolean considerNonVirtual) {
        return sourceOf(port, considerNonVirtual, true);
    }

    private static Optional<EMAPortInstanceSymbol> sourceOf(EMAPortInstanceSymbol port, boolean considerNonVirtual,
                                                            boolean firstCall) {
        if (port.isConstant())
            return Optional.of(port);
        if (port.isOutgoing() && isAtomic(port.getComponentInstance()))
            return Optional.of(port);
        if (port.isIncoming() && !port.getComponentInstance().getParent().isPresent())
            return Optional.of(port);
        if (!firstCall &&
                (!port.getComponentInstance().getParent().isPresent()
                        || considerNonVirtual && isNonVirtual(port.getComponentInstance())))
            return Optional.of(port);

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
        if (sources.size() != 1) {
            if (port.isConfig())
                return Optional.of(port);
            else {
//                Log.warn(String.format("TODO There is no incoming connection for port \"%s\"", port.getFullName()));
                return Optional.empty();
            }
        }

        return sourceOf(sources.get(0).getSourcePort(), considerNonVirtual, false);
    }

    public static boolean isAtomic(EMAComponentInstanceSymbol componentInstance) {
        return componentInstance.getSubComponents().isEmpty() && componentInstance.getConnectorInstances().isEmpty();
    }

    public static boolean isNonVirtual(EMAComponentInstanceSymbol componentInstanceSymbol) {
        return componentInstanceSymbol.isNonVirtual() || !componentInstanceSymbol.getParent().isPresent();
    }
}
