/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.executionOrder;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper;
import de.monticore.lang.monticar.semantics.helper.Find;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper;
import de.monticore.lang.monticar.semantics.loops.detection.StronglyConnectedComponent;
import de.se_rwth.commons.logging.Log;

import java.util.Collections;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.*;

public class ExecutionOrder {

    /**
     * returns true if update cannot be called right after output
     */
    public static boolean needsOutputAndUpdateSplit(EMAComponentInstanceSymbol component) {
        if (component.getOrderOutput().size() == 0) return false;
        if (component.getOrderOutput().size() > 1) return true;
        if (component.getOrderOutput().get(0) + 1 != component.getOrderUpdate()) return true;
        return false;
    }

    /**
     * Sets the values of order_output and order_update for each component
     */
    public static void calculateExecutionOrder(EMAComponentInstanceSymbol rootComponent,
                                               Set<StronglyConnectedComponent> artificialLoops,
                                               boolean handleArtificialLoops) {
        ExecutionOrder order = new ExecutionOrder();

        for (EMAPortInstanceSymbol outport : rootComponent.getOutgoingPortInstances()) {
            Optional<EMAPortInstanceSymbol> sourcePort = ConnectionHelper.sourceOf(outport);
            if (!sourcePort.isPresent()) {
                Log.info(String.format("0xEMAES8712 no connection for component output port \"%s\"", outport.getFullName()), "missing connection");
                continue;
            }
            if (sourcePort.get().isConstant())
                continue;
            if (sourcePort.get().getComponentInstance() == rootComponent)
                continue;
            order.calculateExecutionOrderRec(sourcePort.get().getComponentInstance(), sourcePort.get(), artificialLoops,
                    handleArtificialLoops);
        }

        for (EMAComponentInstanceSymbol component : Find.allAtomicOrNVComponents(rootComponent)) {
            if (component.getOrderOutput().isEmpty())
                order.calculateExecutionOrderRec(component, null, artificialLoops, handleArtificialLoops);
        }

        for (EMAComponentInstanceSymbol component : order.nondfComponents)
            order.setOrderUpdateOfNonDF(component, artificialLoops);
    }

    private Set<EMAComponentInstanceSymbol> handledComponents = new HashSet<>();
    private Set<EMAComponentInstanceSymbol> nondfComponents = new HashSet<>();

    private void calculateExecutionOrderRec(EMAComponentInstanceSymbol component,
                                            EMAPortInstanceSymbol targetsPort,
                                            Set<StronglyConnectedComponent> artificialLoops,
                                            boolean handleArtificialLoops) {
        if (EMAPropertiesHelper.isNonVirtual(component) && !EMAPropertiesHelper.isAtomic(component) && !handledComponents.contains(component))
            calculateExecutionOrder(component, artificialLoops, handleArtificialLoops);

        // handledComponents is only relevant for reoccurring components due to a nondirect feedthrough loop
        //  i.e. a Delay: first time as Output, second time as update
        // For nonvirtual components as part of artificial loop, this is called a fixed number
        // If a component is part of an artificial loop, output could be calculated "too" many times, because
        //  independent components can call this as well, but this does not hurt too much
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

        Set<EMAPortInstanceSymbol> pres = calculatePredecessorPorts(component);

        for (EMAPortInstanceSymbol pre : pres)
            calculateExecutionOrderRec(pre.getComponentInstance(), pre, artificialLoops, handleArtificialLoops);

        int max = pres.stream()
                .map(p -> p.getComponentInstance())
                .map(p -> p.getOrderOutput().isEmpty() ? 0 : Collections.max(p.getOrderOutput())) // max over order output list
                .max(Integer::compareTo) // max over all order outputs
                .orElse(0); // 0 if no pre is present

        component.addOrderOutput(max + 1);
        component.setOrderUpdate(max + 2);
    }

    /**
     * Calculates the set of ports of direct predecessor of a component.
     * A predecessor is a component which the current component depends on (there is a connection from component pre
     *  to this component).
     */
    public static Set<EMAPortInstanceSymbol> calculatePredecessorPorts(EMAComponentInstanceSymbol component) {
        Set<EMAPortInstanceSymbol> pres = new HashSet<>();

        for (EMAPortInstanceSymbol inport : component.getIncomingPortInstances()) {
            Optional<EMAPortInstanceSymbol> sourcePort = ConnectionHelper.sourceOf(inport);
            if (!sourcePort.isPresent()){
//                Log.warn(String.format("TODO There is no incoming connection for port \"%s\"", inport.getFullName()));
                continue;
            }
            if (sourcePort.get().isConstant())
                continue;
            EMAComponentInstanceSymbol componentInstance = sourcePort.get().getComponentInstance();
            if (componentInstance.getParent().isPresent() // do not add for rootcomponent
                    && !(sourcePort.get().isIncoming() && EMAPropertiesHelper.isNonVirtual(componentInstance))) // do not add for nonvirtual components
                pres.add(sourcePort.get());
        }

        return pres;
    }

    /**
     * Calculates the transitive set of all predecessors of a component.
     * A predecessor is a component which the current component depends on (there is a connection from component pre
     *  to this component).
     */
    public static Set<EMAComponentInstanceSymbol> calculatePredecessorsTransitive(EMAComponentInstanceSymbol component) {
        return calculatePredecessorsTransitive(component, new HashSet<>());
    }

    private static Set<EMAComponentInstanceSymbol> calculatePredecessorsTransitive(EMAComponentInstanceSymbol component,
                                                                                   Set<EMAComponentInstanceSymbol> handledComponents) {
        if (handledComponents.contains(component)) return new HashSet<>();
        handledComponents.add(component);

        Set<EMAComponentInstanceSymbol> pres = new HashSet<>();
        Set<EMAComponentInstanceSymbol> directPres = new HashSet<>();

        for (EMAPortInstanceSymbol pre : calculatePredecessorPorts(component))
            directPres.add(pre.getComponentInstance());
        pres.addAll(directPres);

        for (EMAComponentInstanceSymbol pre : directPres)
            pres.addAll(calculatePredecessorsTransitive(pre, handledComponents));

        return pres;
    }

    private void setOrderUpdateOfNonDF(EMAComponentInstanceSymbol component,
                                       Set<StronglyConnectedComponent> artificialLoops) {
        Set<EMAPortInstanceSymbol> pres = calculatePredecessorPorts(component);

        // add self to pre to calculate max order output
        pres.add(component.getOutgoingPortInstances().stream().findAny().get());

        // Pre Output should be calculated by now
        int max = pres.stream()
                .map(p -> p.getComponentInstance())
                .map(p -> Collections.max(p.getOrderOutput())) // max over order output list
                .max(Integer::compareTo) // max over all order outputs
                .orElse(0);  // 0 if no pre is present

        // only + 1 here because component's output is considered already in max
        component.setOrderUpdate(max + 1);
    }

    private boolean isPartOfArtificialLoop(boolean handleArtificialLoops,
                                           EMAComponentInstanceSymbol component,
                                           Set<StronglyConnectedComponent> artificialLoops) {
        if (!handleArtificialLoops) return false;
        if (!EMAPropertiesHelper.isNonVirtual(component) || EMAPropertiesHelper.isAtomic(component))
            return false;

        for (StronglyConnectedComponent artificialLoop : artificialLoops)
            if (artificialLoop.getAllComponents().contains(component))
                return true;

        return false;
    }

    // returns true for output ports of nonvirtual components which only connects to nondirect feedthrough
    //  components inside
    private boolean isNonDirectFeedthroughPort(EMAPortInstanceSymbol port) {
        if (port == null) return false;
        Optional<EMAPortInstanceSymbol> source = sourceOf(port);
        if (!source.isPresent()) {
//            Log.warn(String.format("TODO There is no incoming connection for port \"%s\"", port.getFullName()));
            return true;
        }
        if (source.get().isConstant())
            return true;

        // is direct feedthrough because it points to the nonvirtual parent component and the chain is not
        //  "interrupted" by a nondirect feedthrough component
        if (source.get().isIncoming())
            return false;

        return isPartOfNonDirectFeedthroughChain(source.get().getComponentInstance());
    }

    private boolean isPartOfNonDirectFeedthroughChain(EMAComponentInstanceSymbol component) {
        if (component.isNonDirectFeedThrough()) return true;
        else
            for (EMAPortInstanceSymbol inport : component.getIncomingPortInstances())
                if (!isNonDirectFeedthroughPort(inport))
                    return false;
        return true;
    }
}
