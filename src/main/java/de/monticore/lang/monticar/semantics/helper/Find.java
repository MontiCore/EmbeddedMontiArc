/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnectorInstance;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Optional;
import java.util.stream.Collectors;

import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.*;

public class Find {

    public static LinkedList<EMAComponentInstanceSymbol> allComponents(EMAComponentInstanceSymbol component) {
        LinkedList<EMAComponentInstanceSymbol> result = new LinkedList<>();
        result.add(component);
        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            result.addAll(allComponents(subComponent));
        return result;
    }

    public static LinkedList<EMAComponentInstanceSymbol> allAtomicOrNVComponents(EMAComponentInstanceSymbol component) {
        return allAtomicOrNVComponents(component, true);
    }

    private static LinkedList<EMAComponentInstanceSymbol> allAtomicOrNVComponents(EMAComponentInstanceSymbol component,
                                                                                  boolean firstCall) {
        if ((EMAPropertiesHelper.isAtomic(component) || EMAPropertiesHelper.isNonVirtual(component)) && !firstCall)
            return new LinkedList(Collections.singletonList(component));

        LinkedList<EMAComponentInstanceSymbol> result = new LinkedList<>();
        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            result.addAll(allAtomicOrNVComponents(subComponent, false));
        return result;
    }

    public static LinkedList<EMAComponentInstanceSymbol> allSubSystems(EMAComponentInstanceSymbol component) {
        return allSubSystems(component, true);
    }

    private static LinkedList<EMAComponentInstanceSymbol> allSubSystems(EMAComponentInstanceSymbol component,
                                                                        boolean firstCall) {
        if (EMAPropertiesHelper.isAtomic(component))
            return new LinkedList<>();

        LinkedList<EMAComponentInstanceSymbol> result = new LinkedList<>();
        if (EMAPropertiesHelper.isNonVirtual(component) && !firstCall)
            result.add(component);
        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            result.addAll(allSubSystems(subComponent, false));
        return result;
    }

    public static LinkedList<EMAAtomicConnectorInstance> allAtomicConnectors(EMAComponentInstanceSymbol component) {
        LinkedList<EMAAtomicConnectorInstance> result = new LinkedList<>();

        for (EMAComponentInstanceSymbol subComponent : allAtomicOrNVComponents(component)) {
            subComponent.getOutgoingPortInstances().stream().forEachOrdered(p -> {
                Collection<EMAPortInstanceSymbol> targets = ConnectionHelper.targetsOf(p);
                result.addAll(targets.stream().map(t -> new EMAAtomicConnectorInstance(p, t)).collect(Collectors.toSet()));
            });
            subComponent.getIncomingPortInstances().stream().forEachOrdered(p -> {
                Optional<EMAPortInstanceSymbol> source = sourceOf(p);
                if (source.isPresent())
                    result.add(new EMAAtomicConnectorInstance(source.get(), p));
            });
        }
        for (EMAConnectorInstanceSymbol connector : component.getConnectorInstances()) {
            if (connector.getSourcePort().getComponentInstance().equals(component)
                    && connector.getTargetPort().getComponentInstance().equals(component))
                result.add(new EMAAtomicConnectorInstance(connector.getSourcePort(), connector.getTargetPort()));
            else if (connector.getSourcePort().getComponentInstance().equals(component)
                    && connector.getTargetPort().isConstant())
                result.add(new EMAAtomicConnectorInstance(connector.getSourcePort(), connector.getTargetPort()));
            else if (connector.getSourcePort().isConstant()
                    && connector.getTargetPort().equals(component))
                result.add(new EMAAtomicConnectorInstance(connector.getSourcePort(), connector.getTargetPort()));
        }

        return result;
    }
}
