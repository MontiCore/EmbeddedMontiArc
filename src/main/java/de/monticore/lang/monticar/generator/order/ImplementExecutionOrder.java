/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;

import java.util.*;
import java.util.stream.Collectors;

/*
 *
 *
 */

public class ImplementExecutionOrder {
    private static Map<EMAComponentInstanceSymbol, Collection<EMAPortInstanceSymbol>> dependencies = new HashMap<>();
    private static int s = 0;
    private static int b = 0;

    protected static Map<EMAComponentInstanceSymbol, Collection<EMAConnectorInstanceSymbol>> instanceConnectorsMap = new HashMap<>();
    protected static Map<EMAConnectorInstanceSymbol, EMAPortInstanceSymbol> connectorInstanceSourcePortInstance = new HashMap<>();
    protected static Map<EMAConnectorInstanceSymbol, EMAPortInstanceSymbol> connectorInstanceTargetPortInstance = new HashMap<>();

    protected static Map<EMAPortInstanceSymbol, Collection<EMAConnectorInstanceSymbol>> sourcePortConnectorsList = new HashMap<>();

    /**
     * This function initializes the execution order process. This means the dependencies map will cleared,
     * then the execution order indizes will be zero again.
     * After this the Map 'dependencies' will be filled with the dependent ports for each component again.
     * Then tagExOrder searches for a possible execution order.
     */

    public static List<EMAComponentInstanceSymbol> exOrder(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
        dependencies.clear();

        instanceConnectorsMap.clear();
        connectorInstanceSourcePortInstance.clear();
        connectorInstanceTargetPortInstance.clear();
        sourcePortConnectorsList.clear();

        s = 0;
        b = 0;
        Log.errorIfNull(inst, "The given EMAComponentInstanceSymbol in 'exOrder' is null!");
        getDependencies(inst);
       /* for (EMAComponentInstanceSymbol symbol : dependencies.keySet()) {
            Log.info(symbol.toString(), "KeySet contains:");
        }*/
        EMAComponentInstanceSymbol instTagged = tagExOrder(taggingResolver, inst);
        return getExecutionOrder(taggingResolver, instTagged);
    }

    /**
     * get a map of EMAComponentInstanceSymbols with their dependent ports.
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
     * ***
     */
    private static void getDependencies(EMAComponentInstanceSymbol inst) {
        //Log.info(inst.toString()," getDependencies from:");
//        for (EMAConnectorInstanceSymbol c : inst.getConnectorInstances()) {
        for (EMAConnectorInstanceSymbol c : getAllConnectors(inst)) {
            //Log.info(c.toString(),"ConnectorSymbol:");
            EMAPortInstanceSymbol pt = connectorTargetPort(inst, c);
            EMAPortInstanceSymbol ps = connectorSourcePort(inst, c);
            EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
            EMAComponentInstanceSymbol inst3 = (EMAComponentInstanceSymbol) ps.getEnclosingScope().getSpanningSymbol().get();
            if (!dependencies.containsKey(inst2) && pt.isIncoming()) {
                Collection<EMAPortInstanceSymbol> ports = inst3.getPortInstanceList().stream()
                        .filter(po -> po.equals(ps)).collect(Collectors.toList());
                //Log.info(inst2.toString(), "Added to dependencies");
                dependencies.put(inst2, ports);
            } else if (dependencies.containsKey(inst2) && pt.isIncoming()) {
                Collection<EMAPortInstanceSymbol> ports2 = dependencies.get(inst2);
                ports2.add(ps);
                //Log.info(inst2.toString(), "Added to dependencies already present");
                dependencies.put(inst2, ports2);
            } else {

                Log.info("" + pt.toString(), "Case not handled");
            }
        }
        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
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
     * ***
     */
    private static EMAComponentInstanceSymbol tagExOrder(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {

        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
//		  //Case that the given block is a switch and has a selfloop to port in3
//			if(subInst.getComponentType().getName().equals("SwitchB")
//          || subInst.getComponentType().getName().equals("SwitchM")) {
//			  if(dependencies.containsKey(subInst)) {
//          for (EMAPortInstanceSymbol p : subInst.getOutgoingPorts()) {
//            if (dependencies.get(subInst).contains(p)) {
//              Collection<EMAPortInstanceSymbol> newPorts = dependencies.get(subInst);
//              newPorts.remove(p);
//              dependencies.put(subInst, newPorts);
//            }
//          }
//        }
//      }
            //Case that given block is a block with an initial value parameter such as
            // Constant, Memory or Delay
            if ((subInst.getSubComponents().isEmpty() && subInst.getIncomingPortInstances().isEmpty()
                    || !subInst.getComponentType().getConfigParameters().isEmpty())
                    && taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()) {
                ExecutionOrder e = new NonVirtualBlock(s, b);
                Log.info(e.toString(), "Adding tag:");
                taggingResolver.addTag(subInst, new TagExecutionOrderSymbol(e));
                b += 1;
                Log.info(subInst.toString(), "Instance after Tagging:");
                Collection<EMAConnectorInstanceSymbol> connects = getAllConnectors(inst).stream()
                        .filter(c -> subInst.getPortInstanceList().contains(connectorSourcePort(inst, c)))
                        .collect(Collectors.toList());
                if (!subInst.getComponentType().getConfigParameters().isEmpty()) {
                    dependencies.put(subInst, new ArrayList<EMAPortInstanceSymbol>());
                }
                for (EMAConnectorInstanceSymbol c : connects) {
                    EMAPortInstanceSymbol pt = connectorTargetPort(inst, c);
                    EMAPortInstanceSymbol ps = connectorSourcePort(inst, c);
                    EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
                    if (inst2.getIncomingPortInstances().contains(pt)) {
                        Collection<EMAPortInstanceSymbol> ports = dependencies.get(inst2);
                        ports.remove(ps);
                        dependencies.put(inst2, ports);
                    } else if (inst.getOutgoingPortInstances().contains(pt) && inst.getEnclosingScope()
                            .getSpanningSymbol().isPresent()) {
                        dependencyPortDeletion(taggingResolver,
                                (EMAComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
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
        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
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
     * ***
     */
    private static EMAComponentInstanceSymbol tagExOrderBranch(TaggingResolver taggingResolver, EMAComponentInstanceSymbol subInst, EMAComponentInstanceSymbol inst) {
        if ((taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
                && subInst.getSubComponents().isEmpty()
                && dependencies.get(subInst) != null && dependencies.get(subInst).isEmpty())) {
            ExecutionOrder e = new NonVirtualBlock(s, b);
            taggingResolver.addTag(subInst, new TagExecutionOrderSymbol(e));
            b += 1;

//            System.out.println(subInst.getName());

//            Collection<EMAConnectorInstanceSymbol> connects = getAllConnectors(inst).stream()
//                    .filter(c -> subInst.getOutgoingPortInstances().contains(connectorSourcePort(inst, c)))
//                    .collect(Collectors.toList());
            Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorsWithSourceIn(inst, subInst.getOutgoingPortInstances());

            for (EMAConnectorInstanceSymbol c : connects) {
                EMAPortInstanceSymbol pt = connectorTargetPort(inst, c);
                EMAPortInstanceSymbol ps = connectorSourcePort(inst, c);
                EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
                if (inst2.getIncomingPortInstances().contains(pt)) {
                    Collection<EMAPortInstanceSymbol> ports = dependencies.get(inst2);
                    ports.remove(ps);
                    dependencies.put(inst2, ports);
                } else if (inst.getOutgoingPortInstances().contains(pt) && inst.getEnclosingScope().getSpanningSymbol().isPresent()) {
                    dependencyPortDeletion(taggingResolver, (EMAComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
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
     * ***
     */
    private static EMAComponentInstanceSymbol dependencyPortDeletion(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst, EMAPortInstanceSymbol p) {
//        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectors(inst).stream()
//                .filter(c -> p.equals(connectorSourcePort(inst, c)))
//                .collect(Collectors.toList());

        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorsWithSourceIn(inst, Arrays.asList(p));

        for (EMAConnectorInstanceSymbol c : connects) {
            EMAPortInstanceSymbol pt = connectorTargetPort(inst, c);
            EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
            if (inst2.getIncomingPortInstances().contains(pt)) {
                Collection<EMAPortInstanceSymbol> ports = dependencies.get(inst2);
                ports.remove(p);
                dependencies.put(inst2, ports);
            } else if (inst.getOutgoingPortInstances().contains(pt) && inst.getEnclosingScope()
                    .getSpanningSymbol().isPresent()) {
                dependencyPortDeletion(taggingResolver, (EMAComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
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
     * ***
     */
    private static EMAComponentInstanceSymbol dependencyPortsDeletion(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
//        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectors(inst).stream()
//                .filter(c -> inst.getPortInstanceList().contains(connectorSourcePort(inst, c)))
//                .collect(Collectors.toList());
        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorsWithSourceIn(inst, inst.getPortInstanceList());
        for (EMAConnectorInstanceSymbol c : connects) {
            EMAPortInstanceSymbol pt = connectorTargetPort(inst, c);
            EMAPortInstanceSymbol ps = connectorSourcePort(inst, c);
            EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
            Collection<EMAPortInstanceSymbol> ports = dependencies.get(inst2);
            if (ports == null) {//nullpointer fix
                Log.info(inst2.getName(), "INFO:");
                for (EMAComponentInstanceSymbol instanceSymbol : dependencies.keySet()) {
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
    private static List<EMAComponentInstanceSymbol> getExecutionOrder(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
        Map<ExecutionOrder, EMAComponentInstanceSymbol> exOrder = new HashMap<ExecutionOrder, EMAComponentInstanceSymbol>();
        exOrder = exOrderRecursion(taggingResolver, exOrder, inst);
        List<ExecutionOrder> sortedBlocks = new LinkedList<ExecutionOrder>();
        for (ExecutionOrder o : exOrder.keySet()) {
            sortedBlocks.add(o);
        }
        Collections.sort(sortedBlocks);

        List<EMAComponentInstanceSymbol> orderSorted = new LinkedList<EMAComponentInstanceSymbol>();
        Iterator<ExecutionOrder> it = sortedBlocks.iterator();
        while (it.hasNext()) {
            orderSorted.add(exOrder.get(it.next()));
        }
        return orderSorted;
    }

    /**
     * A recursive Method to get all atomic subComponents of a given component
     */
    private static Map<ExecutionOrder, EMAComponentInstanceSymbol> exOrderRecursion(TaggingResolver taggingResolver,
                                                                                         Map<ExecutionOrder,
                                                                                                 EMAComponentInstanceSymbol> exOrder, EMAComponentInstanceSymbol inst) {
        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
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
    public static EMAPortInstanceSymbol connectorSourcePort(EMAComponentInstanceSymbol inst, EMAConnectorInstanceSymbol c) {

        if(connectorInstanceSourcePortInstance.containsKey(c)){
            return connectorInstanceSourcePortInstance.get(c);
        }

        Iterator<String> parts = Splitters.DOT.split(c.getSource()).iterator();
        Optional<String> instance = Optional.empty();
        Optional<String> instancePort;
        Optional<EMAPortInstanceSymbol> port;
        if (parts.hasNext()) {
            instance = Optional.of(parts.next());
        }
        if (parts.hasNext()) {
            instancePort = Optional.of(parts.next());
            instance = Optional.of(StringUtils.uncapitalize(instance.get()));

            EMAComponentInstanceSymbol inst2 = inst.getSubComponent(instance.get()).get();
            port = inst2.getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
        } else {
            instancePort = instance;

            port = inst.getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
        }

        if (port.isPresent()) {
            connectorInstanceSourcePortInstance.put(c, port.get());
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
    public static EMAPortInstanceSymbol connectorTargetPort(EMAComponentInstanceSymbol inst, EMAConnectorInstanceSymbol c) {

        if(connectorInstanceTargetPortInstance.containsKey(c)){
            return connectorInstanceTargetPortInstance.get(c);
        }

        Iterator<String> parts = Splitters.DOT.split(c.getTarget()).iterator();
        Optional<String> instance = Optional.empty();
        Optional<String> instancePort;
        Optional<EMAPortInstanceSymbol> port;
        if (parts.hasNext()) {
            instance = Optional.of(parts.next());
        }
        if (parts.hasNext()) {
            instancePort = Optional.of(parts.next());
            instance = Optional.of(StringUtils.uncapitalize(instance.get()));
            /*Log.info(instance.get().toString(),"before error");
            for(EMAComponentInstanceSymbol symbol:inst.getSubComponents()){
                Log.info(symbol.toString(),"found:");
            }*/
            EMAComponentInstanceSymbol inst2 = inst.getSubComponent(instance.get()).get();
            port = inst2.getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
        } else {
            instancePort = instance;

            port = inst.getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
        }

        if (port.isPresent()) {
            connectorInstanceTargetPortInstance.put(c, port.get());
            return port.get();
        }

        if (c.getTargetPort() != null) {
            connectorInstanceTargetPortInstance.put(c,c.getTargetPort());
            return c.getTargetPort();
        }
        Log.info(c.getEnclosingScope().toString(), "Scope:");
        Log.info(c.toString(), "Connector:");
        Log.info("False target: " + c.getTarget() + " in: " + c.getEnclosingScope().getName().get(), "ImplementExecutionOrder");
        Log.error("0xAC013 No target have been set for the connector symbol");
        return null;
    }



    public static Collection<EMAConnectorInstanceSymbol> getAllConnectors(EMAComponentInstanceSymbol inst){

        //TODO: Change this method because it will create to many connectors ...
        if(instanceConnectorsMap.containsKey(inst)){
            return instanceConnectorsMap.get(inst);
        }

        Collection<EMAConnectorInstanceSymbol> result;
        if(inst instanceof EMADynamicComponentInstanceSymbol) {
            result = new ArrayList<>();
            for (EMAConnectorInstanceSymbol connector : ((EMADynamicComponentInstanceSymbol)inst).getConnectorInstancesAndEventConnectorInstances()) {
                if (connector instanceof EMADynamicConnectorInstanceSymbol) {
                    EMADynamicConnectorInstanceSymbol d = (EMADynamicConnectorInstanceSymbol) connector;

                    if (d.hasDynamicNew()) {
                        result.addAll(d.getAllPossibleConnectors());
                    } else {
                        result.add(connector);
                    }
                } else {
                    result.add(connector);
                }
            }
        }else{
            result = inst.getConnectorInstances();
        }

        for (EMAConnectorInstanceSymbol con :result) {
            EMAPortInstanceSymbol source = connectorSourcePort(inst, con);
            if(!sourcePortConnectorsList.containsKey(source)){
                sourcePortConnectorsList.put(source, new ArrayList<>());
            }
            if(!sourcePortConnectorsList.get(source).contains(con)) {
                sourcePortConnectorsList.get(source).add(con);
            }
        }

        instanceConnectorsMap.put(inst, result);
        return result;
    }


    public static Collection<EMAConnectorInstanceSymbol> getAllConnectorsWithSourceIn(EMAComponentInstanceSymbol inst, Collection<EMAPortInstanceSymbol> sources){

        List<EMAConnectorInstanceSymbol> result = new ArrayList<>();
        if(!instanceConnectorsMap.containsKey(inst)){
            getAllConnectors(inst);
        }

        for(EMAPortInstanceSymbol s : sources){
            if(sourcePortConnectorsList.containsKey(s)){
                result.addAll(sourcePortConnectorsList.get(s));
            }
        }
        if(!result.isEmpty()) {
            Collections.sort(result, (a, b) -> a.getSourcePosition().compareTo(b.getSourcePosition()));
        }

//        result = getAllConnectors(inst).stream()
//                .filter(c -> sources.contains(connectorSourcePort(inst, c)))
//                .collect(Collectors.toList());

//        System.out.println(result);

        return result;

//        //OLD:
//        Collection<EMAConnectorInstanceSymbol> result;
//
//        return getAllConnectors(inst).stream()
//                .filter(c -> sources.contains(connectorSourcePort(inst, c)))
//                .collect(Collectors.toList());

    }
}

//<editor-fold desc="Wrong idea">

//package de.monticore.lang.monticar.generator.order;
//
//import de.ma2cfg.helper.Names;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
//import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
//import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
//import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderSymbol;
//import de.monticore.lang.tagging._symboltable.TaggingResolver;
//import de.se_rwth.commons.Splitters;
//import de.se_rwth.commons.logging.Log;
//
//import java.util.ArrayList;
//import java.util.Collection;
//import java.util.Collections;
//import java.util.HashMap;
//import java.util.Iterator;
//import java.util.LinkedList;
//import java.util.List;
//import java.util.Map;
//import java.util.Optional;
//import java.util.stream.Collectors;
//
///*
// *
// *
// */
//public class ImplementExecutionOrder {
//    private static Map<EMAComponentInstanceSymbol, Collection<EMAPortInstanceSymbol>> dependencies = new HashMap<>();
//    private static int s = 0;
//    private static int b = 0;
//
//    /**
//     * This function initializes the execution order process. This means the dependencies map will cleared,
//     * then the execution order indizes will be zero again.
//     * After this the Map 'dependencies' will be filled with the dependent ports for each component again.
//     * Then tagExOrder searches for a possible execution order.
//     */
//
//    public static List<EMAComponentInstanceSymbol> exOrder(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
//        dependencies.clear();
//        s = 0;
//        b = 0;
//        Log.errorIfNull(inst, "The given ExpandedComponentInstanceSymbol in 'exOrder' is null!");
//        getDependencies(inst);
//       /* for (ExpandedComponentInstanceSymbol symbol : dependencies.keySet()) {
//            Log.info(symbol.toString(), "KeySet contains:");
//        }*/
//        EMAComponentInstanceSymbol instTagged = tagExOrder(taggingResolver, inst);
//        return getExecutionOrder(taggingResolver, instTagged);
//    }
//
//    /**
//     * get a map of ExpandedComponentInstanceSymbols with their dependent ports.
//     * For example:
//     * component A {
//     * port
//     * in Double in1,
//     * in Double in2,
//     * out Double out1;
//     * <p>
//     * component B {
//     * port
//     * in Double in1,
//     * in Double in2,
//     * out Double out1;
//     * }
//     * connect in1 -> B.in1;
//     * connect in2 -> B.in2;
//     * connect B.out1 -> out1;
//     * }
//     * Then the component B will be stored in 'dependencies' with the ports in1 and in2 of component A.
//     */
//    private static void getDependencies(EMAComponentInstanceSymbol inst) {
//
//
//
//        //Log.info(inst.toString()," getDependencies from:");
//        for (EMAConnectorInstanceSymbol c : getAllConnectorInstances(inst)) {
//            //Log.info(c.toString(),"ConnectorSymbol:");
//            if(c instanceof EMADynamicConnectorInstanceSymbol){
//                EMADynamicConnectorInstanceSymbol d = (EMADynamicConnectorInstanceSymbol)c;
//                if(d.isDynamicSourceNewComponent() || d.isDynamicTargetNewComponent()) {
//                    System.out.println("TADA");
//                    getDependencyForDynamicConnector(inst, d);
//                    continue;
//                }
//
//            }
//            getDependencyForConnector(inst, c);
//        }
//        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
//            if (!subInst.getSubComponents().isEmpty()) {
//                getDependencies(subInst);
//            }
//        }
//    }
//
//    protected static void getDependencyForConnector(EMAComponentInstanceSymbol inst, EMAConnectorInstanceSymbol connector){
//        EMAPortInstanceSymbol pt = connectorTargetPort(inst, connector);
//        EMAPortInstanceSymbol ps = connectorSourcePort(inst, connector);
//        EMAComponentInstanceSymbol instTarget = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
//        EMAComponentInstanceSymbol instSource = (EMAComponentInstanceSymbol) ps.getEnclosingScope().getSpanningSymbol().get();
//        if (!dependencies.containsKey(instTarget) && pt.isIncoming()) {
//            Collection<EMAPortInstanceSymbol> ports = instSource.getPortInstanceList().stream()
//                    .filter(po -> po.equals(ps)).collect(Collectors.toList());
//            //Log.info(instTarget.toString(), "Added to dependencies");
//            dependencies.put(instTarget, ports);
//        } else if (dependencies.containsKey(instTarget) && pt.isIncoming()) {
//            Collection<EMAPortInstanceSymbol> ports2 = dependencies.get(instTarget);
//            ports2.add(ps);
//            //Log.info(instTarget.toString(), "Added to dependencies already present");
//            dependencies.put(instTarget, ports2);
//        } else {
//
//            Log.info("" + pt.toString(), "Case not handled");
//        }
//    }
//
//    protected static void getDependencyForDynamicConnector(EMAComponentInstanceSymbol inst, EMADynamicConnectorInstanceSymbol connector){
//        Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> sources = connector.getSources();
//        Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> targets = connector.getTargets();
//
//        Collection<EMAPortInstanceSymbol> sourcePorts = new ArrayList<>();
//        for(Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> entry : sources.entrySet()){
//            for(EMAPortInstanceSymbol port : entry.getValue()){
//                ((ArrayList<EMAPortInstanceSymbol>) sourcePorts).add(port);
//            }
//        }
//
//        for(Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> entry : targets.entrySet()){
//            if(!entry.getValue().isEmpty() && entry.getValue().get(0).isIncoming()){
//                Collection<EMAPortInstanceSymbol> ports;
//                if(dependencies.containsKey(entry.getKey())){
//                    ports = dependencies.get(entry.getKey());
//                }else{
//                    //add new
//                    ports = new ArrayList<>();
//                }
//
//                for(EMAPortInstanceSymbol portInst : sourcePorts){
//                    if(!ports.contains(portInst)){
//                        ports.add(portInst);
//                    }
//                }
//                dependencies.put(entry.getKey(), ports);
//            }
//        }
//
////        System.out.println(sources);
////        System.out.println(targets);
//    }
//
//    /**
//     * This method tags every component with a TagExecutionOrderSymbol, so after this every subcomponent
//     * has a executionorderTag.
//     *
//     * @param inst The component where the execution order is searched for
//     * @return returns the tagged component
//     */
//    private static EMAComponentInstanceSymbol tagExOrder(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
//
//        Log.info("Tag Ex Order", inst.getName());
//
//        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
////		  //Case that the given block is a switch and has a selfloop to port in3
////			if(subInst.getComponentType().getName().equals("SwitchB")
////          || subInst.getComponentType().getName().equals("SwitchM")) {
////			  if(dependencies.containsKey(subInst)) {
////          for (PortSymbol p : subInst.getOutgoingPorts()) {
////            if (dependencies.get(subInst).contains(p)) {
////              Collection<PortSymbol> newPorts = dependencies.get(subInst);
////              newPorts.remove(p);
////              dependencies.put(subInst, newPorts);
////            }
////          }
////        }
////      }
//            //Case that given block is a block with an initial value parameter such as
//            // Constant, Memory or Delay
//            if ((subInst.getSubComponents().isEmpty() && subInst.getIncomingPortInstances().isEmpty()
//                    || !subInst.getComponentType().getConfigParameters().isEmpty())
//                    && taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()) {
//                ExecutionOrder e = new NonVirtualBlock(s, b);
//                Log.info(e.toString(), "Adding tag:");
//                taggingResolver.addTag(subInst, new TagExecutionOrderSymbol(e));
//                b += 1;
//                Log.info(subInst.toString(), "Instance after Tagging:");
////                Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstances(inst).stream()
////                        .filter(c -> subInst.getPortInstanceList().contains(connectorSourcePort(inst, c)))
////                        .collect(Collectors.toList());
//                Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstancesFilterConnectorSources(inst, subInst.getPortInstanceList());
//                if (!subInst.getComponentType().getConfigParameters().isEmpty()) {
//                    dependencies.put(subInst, new ArrayList<EMAPortInstanceSymbol>());
//                }
//                for (EMAConnectorInstanceSymbol c : connects) {
//                    tagExOrderInnerForConnector(c, taggingResolver, inst);
//                }
//            }
//        }
//
//        //delete dependencies of components which are connected to ports of the outest component
//        if (!inst.getEnclosingScope().getSpanningSymbol().isPresent()) {
//            dependencyPortsDeletion(taggingResolver, inst);
//        }
//
//        //tag components that are newly independent from ports
//        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
//            if (taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
//                    && subInst.getSubComponents().isEmpty()
//                    && (!dependencies.containsKey(subInst) || dependencies.get(subInst).isEmpty())) {
//                tagExOrderBranch(taggingResolver, subInst, inst);
//            } else if (taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
//                    && !subInst.getSubComponents().isEmpty()) {
//                tagExOrder(taggingResolver, subInst);
//            }
//        }
//        return inst;
//    }
//
//    protected static void tagExOrderInnerForConnector(EMAConnectorInstanceSymbol connector, TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst){
//
//        if(connector instanceof EMADynamicConnectorInstanceSymbol){
//            EMADynamicConnectorInstanceSymbol d = (EMADynamicConnectorInstanceSymbol) connector;
//
//            Collection<EMAPortInstanceSymbol> cS = d.getSourcesPorts();
//            Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> mT = d.getTargets();
//            for(Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> tInst : mT.entrySet()){
//                for(EMAPortInstanceSymbol pt : tInst.getValue()){
//                    if (tInst.getKey().getIncomingPortInstances().contains(pt)) {
//                        Collection<EMAPortInstanceSymbol> ports = dependencies.get(tInst.getKey());
//                        ports.removeAll(cS);
//                        dependencies.put(tInst.getKey(), ports);
//                    } else if (inst.getOutgoingPortInstances().contains(pt) && inst.getEnclosingScope().getSpanningSymbol().isPresent()) {
//                        dependencyPortDeletion(taggingResolver, (EMAComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
//                    }
//                }
//            }
//            for(Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> tInst : mT.entrySet()) {
//                if (!tInst.getKey().getSubComponents().isEmpty()) {
//                    for (EMAPortInstanceSymbol pt : tInst.getValue()) {
//                        dependencyPortDeletion(taggingResolver, tInst.getKey(), pt);
//                    }
//                } else {
//                    tagExOrderBranch(taggingResolver, tInst.getKey(), inst);
//                }
//            }
//        } else {
//            Log.info("tagExOrderInnerForConnector: "+connector.toString(),"tagExOrderInnerForConnector");
//            EMAPortInstanceSymbol pt = connectorTargetPort(inst, connector);
//            EMAPortInstanceSymbol ps = connectorSourcePort(inst, connector);
//            EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
//            if (inst2.getIncomingPortInstances().contains(pt)) {
//                Collection<EMAPortInstanceSymbol> ports = dependencies.get(inst2);
//                ports.remove(ps);
//                dependencies.put(inst2, ports);
//            } else if (inst.getOutgoingPortInstances().contains(pt) && inst.getEnclosingScope().getSpanningSymbol().isPresent()) {
//                dependencyPortDeletion(taggingResolver, (EMAComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
//            }
//            if (!inst2.getSubComponents().isEmpty()) {
//                dependencyPortDeletion(taggingResolver, inst2, pt);
//            } else {
//                tagExOrderBranch(taggingResolver, inst2, inst);
//            }
//
//        }
//    }
//
//    /**
//     * if an atomic component got tagged, then first look at components, that are
//     * connected to this component's outputs
//     */
//    private static EMAComponentInstanceSymbol tagExOrderBranch(TaggingResolver taggingResolver, EMAComponentInstanceSymbol subInst, EMAComponentInstanceSymbol inst) {
//        if ((taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
//                && subInst.getSubComponents().isEmpty()
//                && dependencies.get(subInst) != null && dependencies.get(subInst).isEmpty())) {
//            ExecutionOrder e = new NonVirtualBlock(s, b);
//            taggingResolver.addTag(subInst, new TagExecutionOrderSymbol(e));
//            b += 1;
//
//            //TODO: Hier liegt ein fehler:
//            // connectorSourcePort liefert einen falschen source zurück, xyz[1] obwohl dieser gar nicht sein kann!
//            // besseres filtern
////            Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstances(inst);
////            connects = connects.stream()
////                    .filter(c -> subInst.getOutgoingPortInstances().contains(connectorSourcePort(inst, c)))
////                    .collect(Collectors.toList());
//            Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstancesFilterConnectorSources(inst, subInst.getOutgoingPortInstances());
//            for (EMAConnectorInstanceSymbol c : connects) {
//                tagExOrderInnerForConnector(c, taggingResolver, inst);
//            }
//        }
//        return inst;
//    }
//
//    private static EMAComponentInstanceSymbol tagExOrderBranches(TaggingResolver taggingResolver, Collection<EMAComponentInstanceSymbol> subInsts, EMAComponentInstanceSymbol inst){
//
//        Collection<EMAComponentInstanceSymbol> inner = new ArrayList<>();
//
//        //tag them
//        for(EMAComponentInstanceSymbol subInst : subInsts){
//            if ((taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
//                    && subInst.getSubComponents().isEmpty()
//                    && dependencies.get(subInst) != null && dependencies.get(subInst).isEmpty())) {
//                ExecutionOrder e = new NonVirtualBlock(s, b);
//                taggingResolver.addTag(subInst, new TagExecutionOrderSymbol(e));
//                b += 1;
//
//                inner.add(subInst);
//            }
//        }
//
//        return inst;
//    }
//
//    /**
//     * The component of port p got tagged. Now dependencyPortDeletion searches the port where p is
//     * connected to and deletes p out of the dependencies of the enclosing component
//     * of inst.
//     *
//     * @param inst The component which encloses the component of the port p
//     * @param p    The source port
//     */
//    private static EMAComponentInstanceSymbol dependencyPortDeletion(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst, EMAPortInstanceSymbol p) {
////        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstances(inst).stream()
////                .filter(c -> p.equals(connectorSourcePort(inst, c)))
////                .collect(Collectors.toList());
//
//        Collection<EMAPortInstanceSymbol> portC = new ArrayList<>();
//        portC.add(p);
//        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstancesFilterConnectorSources(inst, portC);
//
//        for (EMAConnectorInstanceSymbol c : connects) {
//            if(c instanceof EMADynamicConnectorInstanceSymbol) {
//                EMADynamicConnectorInstanceSymbol d = (EMADynamicConnectorInstanceSymbol) c;
//                Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> mT = d.getTargets();
//                for(Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> tInst : mT.entrySet()) {
//                    for (EMAPortInstanceSymbol pt : tInst.getValue()) {
//                        if (tInst.getKey().getIncomingPortInstances().contains(pt)) {
//                            Collection<EMAPortInstanceSymbol> ports = dependencies.get(tInst.getKey());
//                            ports.remove(p);
//                            dependencies.put(tInst.getKey(), ports);
//                        } else if (inst.getOutgoingPortInstances().contains(pt) && inst.getEnclosingScope().getSpanningSymbol().isPresent()) {
//                            dependencyPortDeletion(taggingResolver, (EMAComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
//                        }
//                    }
//                }
//                for(Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> tInst : mT.entrySet()){
//                    if (tInst.getKey().getSubComponents().isEmpty()) {
//                        tagExOrderBranch(taggingResolver, tInst.getKey(), inst);
//                    } else {
//                        for(EMAPortInstanceSymbol pt : tInst.getValue()) {
//                            dependencyPortDeletion(taggingResolver, tInst.getKey(), pt);
//                        }
//                    }
//                }
//
//            } else {
//                EMAPortInstanceSymbol pt = connectorTargetPort(inst, c);
//                EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
//                if (inst2.getIncomingPortInstances().contains(pt)) {
//                    Collection<EMAPortInstanceSymbol> ports = dependencies.get(inst2);
//                    ports.remove(p);
//                    dependencies.put(inst2, ports);
//                } else if (inst.getOutgoingPortInstances().contains(pt) && inst.getEnclosingScope().getSpanningSymbol().isPresent()) {
//                    dependencyPortDeletion(taggingResolver, (EMAComponentInstanceSymbol) inst.getEnclosingScope().getSpanningSymbol().get(), pt);
//                }
//
//                if (!inst2.getSubComponents().isEmpty()) {
//                    dependencyPortDeletion(taggingResolver, inst2, pt);
//                } else {
//                    tagExOrderBranch(taggingResolver, inst2, inst);
//                }
//            }
//        }
//        return inst;
//    }
//
//    /**
//     * The component inst is the 'outest' component so it is save that all incoming ports of
//     * this component are independent. So all connected components will be independent of these ports
//     *
//     * @param inst The 'outest' component
//     */
//    private static EMAComponentInstanceSymbol dependencyPortsDeletion(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
////        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstances(inst).stream()
////                .filter(c -> inst.getPortInstanceList().contains(connectorSourcePort(inst, c)))
////                .collect(Collectors.toList());
//        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstancesFilterConnectorSources(inst , inst.getPortInstanceList());
////        OLD:
////        for (EMAConnectorInstanceSymbol c : connects) {
////            EMAPortInstanceSymbol pt = connectorTargetPort(inst, c);
////            EMAPortInstanceSymbol ps = connectorSourcePort(inst, c);
////            EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
////            Collection<EMAPortInstanceSymbol> ports = dependencies.get(inst2);
////            if (ports == null) {//nullpointer fix
////                Log.info(inst2.getName(), "INFO:");
////                for (EMAComponentInstanceSymbol instanceSymbol : dependencies.keySet()) {
////                    Log.info(instanceSymbol.getName(), "Available:");
////                }
////                //return inst;
////            } else {
////                ports.remove(ps);
////                dependencies.put(inst2, ports);
////            }
////            if (!inst2.getSubComponents().isEmpty()) {
////                dependencyPortDeletion(taggingResolver, inst2, pt);
////            } else if (inst2.getSubComponents().isEmpty()) {
////                tagExOrderBranch(taggingResolver, inst2, inst);
////            }
////        }
//        for (EMAConnectorInstanceSymbol c : connects) {
//            if(c instanceof EMADynamicConnectorInstanceSymbol){
//                EMADynamicConnectorInstanceSymbol d = (EMADynamicConnectorInstanceSymbol) c;
//                Collection<EMAPortInstanceSymbol> sP = d.getSourcesPorts();
//                Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> tM = d.getTargets();
//
//                for (Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> tInst : tM.entrySet()) {
//                    Collection<EMAPortInstanceSymbol> ports = dependencies.get(tInst.getKey());
//                    if (!ports.isEmpty()) {
//                        ports.removeAll(sP);
//                    }
//                }
//                for (Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> tInst : tM.entrySet()){
//                    if(tInst.getKey().getSubComponents().isEmpty()){
//                        tagExOrderBranch(taggingResolver, tInst.getKey(), inst);
//                    }else{
//                        for(EMAPortInstanceSymbol pI : tInst.getValue()) {
//                            dependencyPortDeletion(taggingResolver, tInst.getKey(), pI);
//                        }
//                    }
//                }
//            }else {
//                // Use old method
//                EMAPortInstanceSymbol pt = connectorTargetPort(inst, c);
//                EMAPortInstanceSymbol ps = connectorSourcePort(inst, c);
//                EMAComponentInstanceSymbol inst2 = (EMAComponentInstanceSymbol) pt.getEnclosingScope().getSpanningSymbol().get();
//                Collection<EMAPortInstanceSymbol> ports = dependencies.get(inst2);
//                if (ports == null) {//nullpointer fix
//                    Log.info(inst2.getName(), "INFO:");
//                    for (EMAComponentInstanceSymbol instanceSymbol : dependencies.keySet()) {
//                        Log.info(instanceSymbol.getName(), "Available:");
//                    }
//                    //return inst;
//                } else {
//                    ports.remove(ps);
//                    dependencies.put(inst2, ports);
//                }
//                if (!inst2.getSubComponents().isEmpty()) {
//                    dependencyPortDeletion(taggingResolver, inst2, pt);
//                } else if (inst2.getSubComponents().isEmpty()) {
//                    tagExOrderBranch(taggingResolver, inst2, inst);
//                }
//            }
//        }
//        return inst;
//    }
//
//    /**
//     * This method saves all atomic subcomponents of inst with their 'NonVirtualBlocks' in 'exOrder'.
//     * Then the Non VirtualBlocks will be saved and sorted in a List. In this sorted order the associated
//     * components will be stored in a list too.
//     *
//     * @param inst
//     * @return A list which contains all atomic subComponents of inst in a sorted order regarding the NonvirtualBlocks
//     */
//    private static List<EMAComponentInstanceSymbol> getExecutionOrder(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
//        Map<ExecutionOrder, EMAComponentInstanceSymbol> exOrder = new HashMap<ExecutionOrder, EMAComponentInstanceSymbol>();
//        exOrder = exOrderRecursion(taggingResolver, exOrder, inst);
//        List<ExecutionOrder> sortedBlocks = new LinkedList<ExecutionOrder>();
//        for (ExecutionOrder o : exOrder.keySet()) {
//            sortedBlocks.add(o);
//        }
//        Collections.sort(sortedBlocks);
//
//        List<EMAComponentInstanceSymbol> orderSorted = new LinkedList<EMAComponentInstanceSymbol>();
//        Iterator<ExecutionOrder> it = sortedBlocks.iterator();
//        while (it.hasNext()) {
//            orderSorted.add(exOrder.get(it.next()));
//        }
//        return orderSorted;
//    }
//
//    /**
//     * A recursive Method to get all atomic subComponents of a given component
//     */
//    private static Map<ExecutionOrder, EMAComponentInstanceSymbol> exOrderRecursion(TaggingResolver taggingResolver,
//                                                                                    Map<ExecutionOrder,EMAComponentInstanceSymbol> exOrder,
//                                                                                    EMAComponentInstanceSymbol inst) {
//        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
//            Log.info(taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).size() + "",
//                    "Amount of ExecutionOrder Tags");
//            if (taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).size() == 1) {
//                exOrder.put(((TagExecutionOrderSymbol) taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND)
//                        .iterator().next()).getExecutionOrder(), subInst);
//            } else {
//                exOrderRecursion(taggingResolver, exOrder, subInst);
//            }
//        }
//        return exOrder;
//    }
//
//    /**
//     * A method to get the source port of a given connector
//     *
//     * @param inst The enclosing component
//     * @param c    The given connector
//     * @return Source port of c
//     */
//    public static EMAPortInstanceSymbol connectorSourcePort(EMAComponentInstanceSymbol inst, EMAConnectorInstanceSymbol c) {
//        Iterator<String> parts = Splitters.DOT.split(c.getSource()).iterator();
//        Optional<String> instance = Optional.empty();
//        Optional<String> instancePort;
//        Optional<EMAPortInstanceSymbol> port = Optional.empty();
//        if (parts.hasNext()) {
//            instance = Optional.of(parts.next());
//        }
//        if (parts.hasNext()) {
//            instancePort = Optional.of(parts.next());
//            instance = Optional.of(Names.FirstLowerCase(instance.get()));
//
//            // Old:
////            EMAComponentInstanceSymbol inst2 = inst.getSubComponent(instance.get()).get();
////            port = inst2.getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
//
//            Optional<EMAComponentInstanceSymbol> inst2 = inst.getSubComponent(instance.get());
//            if(inst2.isPresent()) {
//                port = inst2.get().getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
//            }
//        } else {
//            instancePort = instance;
//
//            port = inst.getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
//        }
//
//        if (port.isPresent()) {
//            return port.get();
//        }
//
//        if(c.getSourcePort() != null){
//            return c.getSourcePort();
//        }
//
//        Log.info("ImplementExecutionOrder", "False Source: " + c.getSource() + " in: " + c.getEnclosingScope().getName().get());
//        Log.error("0xAC012 No source have been set for the connector symbol");
//        return null;
//    }
//
//    /**
//     * A method to get the target port of a given connector
//     *
//     * @param inst The enclosing component
//     * @param c    The given connector
//     * @return Target port of c
//     */
//    public static EMAPortInstanceSymbol connectorTargetPort(EMAComponentInstanceSymbol inst, EMAConnectorInstanceSymbol c) {
//        Iterator<String> parts = Splitters.DOT.split(c.getTarget()).iterator();
//        Optional<String> instance = Optional.empty();
//        Optional<String> instancePort;
//        Optional<EMAPortInstanceSymbol> port = Optional.empty();
//        if (parts.hasNext()) {
//            instance = Optional.of(parts.next());
//        }
//        if (parts.hasNext()) {
//            instancePort = Optional.of(parts.next());
//            instance = Optional.of(Names.FirstLowerCase(instance.get()));
//            /*Log.info(instance.get().toString(),"before error");
//            for(ExpandedComponentInstanceSymbol symbol:inst.getSubComponents()){
//                Log.info(symbol.toString(),"found:");
//            }*/
//            //Old:
////            EMAComponentInstanceSymbol inst2 = inst.getSubComponent(instance.get()).get();
////            port = inst2.getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
//
//            Optional<EMAComponentInstanceSymbol> inst2 = inst.getSubComponent(instance.get());
//            if(inst2.isPresent()) {
//                port = inst2.get().getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
//            }
//        } else {
//            instancePort = instance;
//
//            port = inst.getSpannedScope().<EMAPortInstanceSymbol>resolve(instancePort.get(), EMAPortInstanceSymbol.KIND);
//        }
//
//        if (port.isPresent()) {
//            return port.get();
//        }
//
//        if (c.getTargetPort() != null) {
//            return c.getTargetPort();
//        }
//        Log.info(c.getEnclosingScope().toString(), "Scope:");
//        Log.info(c.toString(), "Connector:");
//        Log.info("False target: " + c.getTarget() + " in: " + c.getEnclosingScope().getName().get(), "ImplementExecutionOrder");
//        Log.error("0xAC013 No target have been set for the connector symbol");
//        return null;
//    }
//
//
//    /**
//     * A wrapper method that returns ALL connectors of a given Instance
//     * (This includes the connectors inside event handlers!)
//     * @param inst
//     * @return
//     */
//    public static Collection<EMAConnectorInstanceSymbol> getAllConnectorInstances(EMAComponentInstanceSymbol inst){
//        if(inst instanceof EMADynamicComponentInstanceSymbol){
//            return ((EMADynamicComponentInstanceSymbol)inst).getConnectorInstancesAndEventConnectorInstances();
//        }else{
//            return inst.getConnectorInstances();
//        }
//    }
//
//    public static Collection<EMAConnectorInstanceSymbol> getAllConnectorInstancesFilterConnectorSources(EMAComponentInstanceSymbol inst, Collection<EMAPortInstanceSymbol> portsTest){
//        Collection<EMAConnectorInstanceSymbol> connects = getAllConnectorInstances(inst);
////        connects = connects.stream()
////                .filter(c -> portsTest.contains(connectorSourcePort(inst, c)))
////                .collect(Collectors.toList());
//        Collection<EMAConnectorInstanceSymbol> result = new ArrayList<>();
//
//        for (EMAConnectorInstanceSymbol con : connects){
//            if(con instanceof EMADynamicConnectorInstanceSymbol){
//                EMADynamicConnectorInstanceSymbol d = (EMADynamicConnectorInstanceSymbol) con;
//                Collection<EMAPortInstanceSymbol> sources = d.getSourcesPorts();
//
//                for(EMAPortInstanceSymbol sP : sources){
//                    if(portsTest.contains(sP)){
//                        result.add(con);
//                        break;
//                    }
//                }
//
//            }else{
//                if(portsTest.contains(connectorSourcePort(inst, con))){
//                    result.add(con);
//                }
//            }
//        }
//
//        return result;
//    }
//}

// </editor-fold>
