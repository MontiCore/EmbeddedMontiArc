/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.executionOrder;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper;
import de.monticore.lang.monticar.semantics.loops.detection.StronglyConnectedComponent;
import de.se_rwth.commons.logging.Log;

import java.util.Collections;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.isNonVirtual;
import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.sourceOf;

public class ExecutionOrder {

    public static void calculateExecutionOrder(EMAComponentInstanceSymbol rootComponent,
                                               Set<StronglyConnectedComponent> artificialLoops,
                                               boolean handleArtificialLoops) {
        ExecutionOrder order = new ExecutionOrder();

        for (EMAPortInstanceSymbol outport : rootComponent.getOutgoingPortInstances()) {
            Optional<EMAPortInstanceSymbol> sourcePort = ConnectionHelper.sourceOf(outport);
            if (!sourcePort.isPresent())
                Log.error("TODO no connection for component output port");
            order.calculateExecutionOrderRec(sourcePort.get().getComponentInstance(), sourcePort.get(), artificialLoops,
                    handleArtificialLoops);
        }
        for (EMAComponentInstanceSymbol component : order.nondfComponents) {
            order.setOrderUpdateNonDF(component, artificialLoops);
        }
    }

    public static boolean needsOutputAndUpdateSplit(EMAComponentInstanceSymbol component) {
        if (component.getOrderOutput().size() == 0) return false;
        if (component.getOrderOutput().size() > 1) return true;
        if (component.getOrderOutput().get(0) + 1 != component.getOrderUpdate()) return true;
        return false;
    }

    private Set<EMAComponentInstanceSymbol> handledComponents = new HashSet<>();
    private Set<EMAComponentInstanceSymbol> nondfComponents = new HashSet<>();

    private void calculateExecutionOrderRec(EMAComponentInstanceSymbol component,
                                            EMAPortInstanceSymbol targetsPort, Set<StronglyConnectedComponent> artificialLoops,
                                            boolean handleArtificialLoops) {
        if (isNonVirtual(component) && !ConnectionHelper.isAtomic(component) && !handledComponents.contains(component))
            calculateExecutionOrder(component, artificialLoops, handleArtificialLoops);

        // handledComponents is only relevant for reoccurring components due to a nondirect feedthrough loop
        //  i.e. a Delay: first time as Output, second time as update
        // For nonvirtual components as part of artificial loop, this is called a fixed number
        if (isPartOfArtificialLoop(handleArtificialLoops, component, artificialLoops)) {
            if (isNonDirectFeedthroughPort(targetsPort)) {
                nondfComponents.add(component);
                component.setOrderOutput(Collections.singletonList(1));
                return;
            }
        } else if (handledComponents.contains(component))
            return;

        handledComponents.add(component);

        if (component.isNonDirectFeedThrough()) {
            nondfComponents.add(component);
            component.setOrderOutput(Collections.singletonList(1));
            return;
        }

        Set<EMAPortInstanceSymbol> pres = calcPredecessors(component);

        for (EMAPortInstanceSymbol pre : pres)
            calculateExecutionOrderRec(pre.getComponentInstance(), pre, artificialLoops, handleArtificialLoops);

        int max = pres.stream()
                .map(p -> p.getComponentInstance())
                .map(p -> p.getOrderOutput().isEmpty() ? 0 : Collections.max(p.getOrderOutput())) // max over order output list
                .mapToInt(v -> v)
                .max() // max over all order outputs
                .orElse(0); // 0 if no pre is present

        component.addOrderOutput(max + 1);
        component.setOrderUpdate(max + 2);
    }

    private Set<EMAPortInstanceSymbol> calcPredecessors(EMAComponentInstanceSymbol component) {
        Set<EMAPortInstanceSymbol> pres = new HashSet<>();
        for (EMAPortInstanceSymbol inport : component.getIncomingPortInstances()) {
            Optional<EMAPortInstanceSymbol> sourcePort = ConnectionHelper.sourceOf(inport);
            if (!sourcePort.isPresent())
                Log.error("TODO no connection for component input port");
            EMAComponentInstanceSymbol componentInstance = sourcePort.get().getComponentInstance();
            if (componentInstance.getParent().isPresent() &&
                    !(sourcePort.get().isIncoming() && isNonVirtual(componentInstance)))
                pres.add(sourcePort.get());
        }
        return pres;
    }

    private void setOrderUpdateNonDF(EMAComponentInstanceSymbol component,
                                     Set<StronglyConnectedComponent> artificialLoops) {
        Set<EMAPortInstanceSymbol> pres = calcPredecessors(component);

        // add self to pre to calculate max order output
        pres.add(component.getOutgoingPortInstances().stream().findAny().get());

        // Pre Output should be calculated by now
        int max = pres.stream()
                .map(p -> p.getComponentInstance())
                .map(p -> Collections.max(p.getOrderOutput())) // max over order output list
                .mapToInt(v -> v)
                .max() // max over all order outputs
                .orElse(0);  // 0 if no pre is present

        // only + 1 here because component's output is considered already in max
        component.setOrderUpdate(max + 1);
    }

    private boolean isPartOfArtificialLoop(boolean handleArtificialLoops, EMAComponentInstanceSymbol component,
                                           Set<StronglyConnectedComponent> artificialLoops) {
        if (!handleArtificialLoops) return false;
        if (!isNonVirtual(component) || ConnectionHelper.isAtomic(component))
            return false;

        for (StronglyConnectedComponent artificialLoop : artificialLoops)
            if (artificialLoop.getAllComponents().contains(component))
                return true;

        return false;
    }

    private boolean isNonDirectFeedthroughPort(EMAPortInstanceSymbol port) {
        return isPartOfNonDirectFeedthroughChain(sourceOf(port).get().getComponentInstance());
    }

    private boolean isPartOfNonDirectFeedthroughChain(EMAComponentInstanceSymbol component) {
        if (component.isNonDirectFeedThrough()) return true;
        else
            for (EMAPortInstanceSymbol inport : component.getIncomingPortInstances()) {
                Optional<EMAPortInstanceSymbol> source = sourceOf(inport);
                if (!source.isPresent())
                    Log.error("TODO no source port");
                if (source.get().isIncoming()) // is direct feedthrough
                    return false;
                if (!isPartOfNonDirectFeedthroughChain(source.get().getComponentInstance()))
                    return false;
            }
        return true;
    }
}
