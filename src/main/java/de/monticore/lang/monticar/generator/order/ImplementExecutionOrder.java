package de.monticore.lang.monticar.generator.order;

import de.ma2cfg.helper.Names;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

/*
 *
 *  @author ernst
 *
 */
public class ImplementExecutionOrder {
    private static Map<ExpandedComponentInstanceSymbol, Collection<PortSymbol>> dependencies = new HashMap<>();
    private static int s = 0;
    private static int b = 0;

    /**
     * This function initializes the execution order process. This means the dependencies map will cleared,
     * then the execution order indizes will be zero again.
     * After this the Map 'dependencies' will be filled with the dependent ports for each component again.
     * Then tagExOrder searches for a possible execution order.
     */

    public static List<ExpandedComponentInstanceSymbol> exOrder(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol inst) {
        dependencies.clear();
        s = 0;
        b = 0;
        Log.errorIfNull(inst, "The given ExpandedComponentInstanceSymbol in 'exOrder' is null!");
        getDependencies(inst);
       /* for (ExpandedComponentInstanceSymbol symbol : dependencies.keySet()) {
            Log.info(symbol.toString(), "KeySet contains:");
        }*/
        ExpandedComponentInstanceSymbol instTagged = tagExOrder(taggingResolver, inst);
        return getExecutionOrder(taggingResolver, instTagged);
    }

    /**
     * get a map of ExpandedComponentInstanceSymbols with their dependent ports.
     * For example:
     * component A {
     * port
     * in Double in1,
     * in Double in2,
     * out Double out1;
     * <p>
     * component B {
     * port
     * in Double in1,
     * in Double in2,
     * out Double out1;
     * }
     * connect in1 -> B.in1;
     * connect in2 -> B.in2;
     * connect B.out1 -> out1;
     * }
     * Then the component B will be stored in 'dependencies' with the ports in1 and in2 of component A.
     */
    private static void getDependencies(ExpandedComponentInstanceSymbol inst) {
        //Log.info(inst.toString()," getDependencies from:");
        for (ConnectorSymbol c : inst.getConnectors()) {
            //Log.info(c.toString(),"ConnectorSymbol:");
            PortSymbol pt = connectorTargetPort(inst, c);
            PortSymbol ps = connectorSourcePort(inst, c);
            ExpandedComponentInstanceSymbol inst2 = (ExpandedComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
            ExpandedComponentInstanceSymbol inst3 = (ExpandedComponentInstanceSymbol) ps.getEnclosingScope().getSpanningSymbol().get();
            if (!dependencies.containsKey(inst2) && pt.isIncoming()) {
                Collection<PortSymbol> ports = inst3.getPorts().stream()
                        .filter(po -> po.equals(ps)).collect(Collectors.toList());
                //Log.info(inst2.toString(), "Added to dependencies");
                dependencies.put(inst2, ports);
            } else if (dependencies.containsKey(inst2) && pt.isIncoming()) {
                Collection<PortSymbol> ports2 = dependencies.get(inst2);
                ports2.add(ps);
                //Log.info(inst2.toString(), "Added to dependencies already present");
                dependencies.put(inst2, ports2);
            } else {

                Log.info("" + pt.toString(), "Case not handled");
            }
        }
        for (ExpandedComponentInstanceSymbol subInst : inst.getSubComponents()) {
            if (!subInst.getSubComponents().isEmpty()) {
                getDependencies(subInst);
            }
        }
    }

    /**
     * This method tags every component with a TagExecutionOrderSymbol, so after this every subcomponent
     * has a executionorderTag.
     *
     * @param inst The component where the execution order is searched for
     * @return returns the tagged component
     */
    private static ExpandedComponentInstanceSymbol tagExOrder(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol inst) {

        for (ExpandedComponentInstanceSymbol subInst : inst.getSubComponents()) {
//		  //Case that the given block is a switch and has a selfloop to port in3
//			if(subInst.getComponentType().getName().equals("SwitchB")
//          || subInst.getComponentType().getName().equals("SwitchM")) {
//			  if(dependencies.containsKey(subInst)) {
//          for (PortSymbol p : subInst.getOutgoingPorts()) {
//            if (dependencies.get(subInst).contains(p)) {
//              Collection<PortSymbol> newPorts = dependencies.get(subInst);
//              newPorts.remove(p);
//              dependencies.put(subInst, newPorts);
//            }
//          }
//        }
//      }
            //Case that given block is a block with an initial value parameter such as
            // Constant, Memory or Delay
            if ((subInst.getSubComponents().isEmpty() && subInst.getIncomingPorts().isEmpty()
                    || !subInst.getComponentType().getConfigParameters().isEmpty())
                    && taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()) {
                ExecutionOrder e = new NonVirtualBlock(s, b);
                Log.info(e.toString(), "Adding tag:");
                taggingResolver.addTag(subInst, new TagExecutionOrderSymbol(e));
                b += 1;
                Log.info(subInst.toString(), "Instance after Tagging:");
                Collection<ConnectorSymbol> connects = inst.getConnectors().stream()
                        .filter(c -> subInst.getPorts().contains(connectorSourcePort(inst, c)))
                        .collect(Collectors.toList());
                if (!subInst.getComponentType().getConfigParameters().isEmpty()) {
                    dependencies.put(subInst, new ArrayList<PortSymbol>());
                }
                for (ConnectorSymbol c : connects) {
                    PortSymbol pt = connectorTargetPort(inst, c);
                    PortSymbol ps = connectorSourcePort(inst, c);
                    ExpandedComponentInstanceSymbol inst2 = (ExpandedComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
                    if (inst2.getIncomingPorts().contains(pt)) {
                        Collection<PortSymbol> ports = dependencies.get(inst2);
                        ports.remove(ps);
                        dependencies.put(inst2, ports);
                    } else if (inst.getOutgoingPorts().contains(pt) && inst.getEnclosingScope()
                            .getSpanningSymbol().isPresent()) {
                        dependencyPortDeletion(taggingResolver,
                                (ExpandedComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
                    }
                    //delete dependency of port in target component
                    if (!inst2.getSubComponents().isEmpty()) {
                        dependencyPortDeletion(taggingResolver, inst2, pt);
                    } else {
                        tagExOrderBranch(taggingResolver, inst2, inst);
                    }
                }
            }
        }

        //delete dependencies of components which are connected to ports of the outest component
        if (!inst.getEnclosingScope().getSpanningSymbol().isPresent()) {
            dependencyPortsDeletion(taggingResolver, inst);
        }

        //tag components that are newly independent from ports
        for (ExpandedComponentInstanceSymbol subInst : inst.getSubComponents()) {
            if (taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
                    && subInst.getSubComponents().isEmpty()
                    && (!dependencies.containsKey(subInst) || dependencies.get(subInst).isEmpty())) {
                tagExOrderBranch(taggingResolver, subInst, inst);
            } else if (taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
                    && !subInst.getSubComponents().isEmpty()) {
                tagExOrder(taggingResolver, subInst);
            }
        }
        return inst;
    }

    /**
     * if an atomic component got tagged, then first look at components, that are
     * connected to this component's outputs
     */
    private static ExpandedComponentInstanceSymbol tagExOrderBranch(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol subInst, ExpandedComponentInstanceSymbol inst) {
        if ((taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
                && subInst.getSubComponents().isEmpty()
                && dependencies.get(subInst) != null && dependencies.get(subInst).isEmpty())) {
            ExecutionOrder e = new NonVirtualBlock(s, b);
            taggingResolver.addTag(subInst, new TagExecutionOrderSymbol(e));
            b += 1;
            Collection<ConnectorSymbol> connects = inst.getConnectors().stream()
                    .filter(c -> subInst.getOutgoingPorts().contains(connectorSourcePort(inst, c)))
                    .collect(Collectors.toList());
            for (ConnectorSymbol c : connects) {
                PortSymbol pt = connectorTargetPort(inst, c);
                PortSymbol ps = connectorSourcePort(inst, c);
                ExpandedComponentInstanceSymbol inst2 = (ExpandedComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
                if (inst2.getIncomingPorts().contains(pt)) {
                    Collection<PortSymbol> ports = dependencies.get(inst2);
                    ports.remove(ps);
                    dependencies.put(inst2, ports);
                } else if (inst.getOutgoingPorts().contains(pt) && inst.getEnclosingScope().getSpanningSymbol().isPresent()) {
                    dependencyPortDeletion(taggingResolver, (ExpandedComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
                }
                if (!inst2.getSubComponents().isEmpty()) {
                    dependencyPortDeletion(taggingResolver, inst2, pt);
                } else if (inst2.getSubComponents().isEmpty()) {
                    tagExOrderBranch(taggingResolver, inst2, inst);
                }
            }
        }
        return inst;
    }

    /**
     * The component of port p got tagged. Now dependencyPortDeletion searches the port where p is
     * connected to and deletes p out of the dependencies of the enclosing component
     * of inst.
     *
     * @param inst The component which encloses the component of the port p
     * @param p    The source port
     */
    private static ExpandedComponentInstanceSymbol dependencyPortDeletion(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol inst, PortSymbol p) {
        Collection<ConnectorSymbol> connects = inst.getConnectors().stream()
                .filter(c -> p.equals(connectorSourcePort(inst, c)))
                .collect(Collectors.toList());
        for (ConnectorSymbol c : connects) {
            PortSymbol pt = connectorTargetPort(inst, c);
            ExpandedComponentInstanceSymbol inst2 = (ExpandedComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
            if (inst2.getIncomingPorts().contains(pt)) {
                Collection<PortSymbol> ports = dependencies.get(inst2);
                ports.remove(p);
                dependencies.put(inst2, ports);
            } else if (inst.getOutgoingPorts().contains(pt) && inst.getEnclosingScope()
                    .getSpanningSymbol().isPresent()) {
                dependencyPortDeletion(taggingResolver, (ExpandedComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
            }

            if (!inst2.getSubComponents().isEmpty()) {
                dependencyPortDeletion(taggingResolver, inst2, pt);
            } else if (inst2.getSubComponents().isEmpty()) {
                tagExOrderBranch(taggingResolver, inst2, inst);
            }
        }
        return inst;
    }

    /**
     * The component inst is the 'outest' component so it is save that all incoming ports of
     * this component are independent. So all connected components will be independent of these ports
     *
     * @param inst The 'outest' component
     */
    private static ExpandedComponentInstanceSymbol dependencyPortsDeletion(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol inst) {
        Collection<ConnectorSymbol> connects = inst.getConnectors().stream()
                .filter(c -> inst.getPorts().contains(connectorSourcePort(inst, c)))
                .collect(Collectors.toList());
        for (ConnectorSymbol c : connects) {
            PortSymbol pt = connectorTargetPort(inst, c);
            PortSymbol ps = connectorSourcePort(inst, c);
            ExpandedComponentInstanceSymbol inst2 = (ExpandedComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
            Collection<PortSymbol> ports = dependencies.get(inst2);
            if (ports == null) {//nullpointer fix
                Log.info(inst2.getName(), "INFO:");
                for (ExpandedComponentInstanceSymbol instanceSymbol : dependencies.keySet()) {
                    Log.info(instanceSymbol.getName(), "Available:");
                }
                //return inst;
            } else {
                ports.remove(ps);
                dependencies.put(inst2, ports);
            }
            if (!inst2.getSubComponents().isEmpty()) {
                dependencyPortDeletion(taggingResolver, inst2, pt);
            } else if (inst2.getSubComponents().isEmpty()) {
                tagExOrderBranch(taggingResolver, inst2, inst);
            }
        }
        return inst;
    }

    /**
     * This method saves all atomic subcomponents of inst with their 'NonVirtualBlocks' in 'exOrder'.
     * Then the Non VirtualBlocks will be saved and sorted in a List. In this sorted order the associated
     * components will be stored in a list too.
     *
     * @param inst
     * @return A list which contains all atomic subComponents of inst in a sorted order regarding the NonvirtualBlocks
     */
    private static List<ExpandedComponentInstanceSymbol> getExecutionOrder(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol inst) {
        Map<ExecutionOrder, ExpandedComponentInstanceSymbol> exOrder = new HashMap<ExecutionOrder, ExpandedComponentInstanceSymbol>();
        exOrder = exOrderRecursion(taggingResolver, exOrder, inst);
        List<ExecutionOrder> sortedBlocks = new LinkedList<ExecutionOrder>();
        for (ExecutionOrder o : exOrder.keySet()) {
            sortedBlocks.add(o);
        }
        Collections.sort(sortedBlocks);

        List<ExpandedComponentInstanceSymbol> orderSorted = new LinkedList<ExpandedComponentInstanceSymbol>();
        Iterator<ExecutionOrder> it = sortedBlocks.iterator();
        while (it.hasNext()) {
            orderSorted.add(exOrder.get(it.next()));
        }
        return orderSorted;
    }

    /**
     * A recursive Method to get all atomic subComponents of a given component
     */
    private static Map<ExecutionOrder, ExpandedComponentInstanceSymbol> exOrderRecursion(TaggingResolver taggingResolver,
                                                                                         Map<ExecutionOrder,
                                                                                                 ExpandedComponentInstanceSymbol> exOrder, ExpandedComponentInstanceSymbol inst) {
        for (ExpandedComponentInstanceSymbol subInst : inst.getSubComponents()) {
            Log.info(taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).size() + "",
                    "Amount of ExecutionOrder Tags");
            if (taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).size() == 1) {
                exOrder.put(((TagExecutionOrderSymbol) taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND)
                        .iterator().next()).getExecutionOrder(), subInst);
            } else {
                exOrderRecursion(taggingResolver, exOrder, subInst);
            }
        }
        return exOrder;
    }

    /**
     * A method to get the source port of a given connector
     *
     * @param inst The enclosing component
     * @param c    The given connector
     * @return Source port of c
     */
    public static PortSymbol connectorSourcePort(ExpandedComponentInstanceSymbol inst, ConnectorSymbol c) {
        Iterator<String> parts = Splitters.DOT.split(c.getSource()).iterator();
        Optional<String> instance = Optional.empty();
        Optional<String> instancePort;
        Optional<PortSymbol> port;
        if (parts.hasNext()) {
            instance = Optional.of(parts.next());
        }
        if (parts.hasNext()) {
            instancePort = Optional.of(parts.next());
            instance = Optional.of(Names.FirstLowerCase(instance.get()));

            ExpandedComponentInstanceSymbol inst2 = inst.getSubComponent(instance.get()).get();
            port = inst2.getSpannedScope().<PortSymbol>resolve(instancePort.get(), PortSymbol.KIND);
        } else {
            instancePort = instance;

            port = inst.getSpannedScope().<PortSymbol>resolve(instancePort.get(), PortSymbol.KIND);
        }

        if (port.isPresent()) {
            return port.get();
        }

        Log.info("ImplementExecutionOrder", "False Source: " + c.getSource() + " in: " + c.getEnclosingScope().getName().get());
        Log.error("0xAC012 No source have been set for the connector symbol");
        return null;
    }

    /**
     * A method to get the target port of a given connector
     *
     * @param inst The enclosing component
     * @param c    The given connector
     * @return Target port of c
     */
    public static PortSymbol connectorTargetPort(ExpandedComponentInstanceSymbol inst, ConnectorSymbol c) {
        Iterator<String> parts = Splitters.DOT.split(c.getTarget()).iterator();
        Optional<String> instance = Optional.empty();
        Optional<String> instancePort;
        Optional<PortSymbol> port;
        if (parts.hasNext()) {
            instance = Optional.of(parts.next());
        }
        if (parts.hasNext()) {
            instancePort = Optional.of(parts.next());
            instance = Optional.of(Names.FirstLowerCase(instance.get()));
            /*Log.info(instance.get().toString(),"before error");
            for(ExpandedComponentInstanceSymbol symbol:inst.getSubComponents()){
                Log.info(symbol.toString(),"found:");
            }*/
            ExpandedComponentInstanceSymbol inst2 = inst.getSubComponent(instance.get()).get();
            port = inst2.getSpannedScope().<PortSymbol>resolve(instancePort.get(), PortSymbol.KIND);
        } else {
            instancePort = instance;

            port = inst.getSpannedScope().<PortSymbol>resolve(instancePort.get(), PortSymbol.KIND);
        }

        if (port.isPresent()) {
            return port.get();
        }

        if (c.getTargetPort() != null) {
            return c.getTargetPort();
        }
        Log.info(c.getEnclosingScope().toString(), "Scope:");
        Log.info(c.toString(), "Connector:");
        Log.info("False target: " + c.getTarget() + " in: " + c.getEnclosingScope().getName().get(), "ImplementExecutionOrder");
        Log.error("0xAC013 No target have been set for the connector symbol");
        return null;
    }
}
