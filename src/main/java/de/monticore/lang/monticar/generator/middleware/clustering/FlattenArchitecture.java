package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.*;
import java.util.stream.Collectors;

public class FlattenArchitecture {

    /**
     * Put all computational components in the top level component and preserve all connectors between these components.
     * Try to preserve component and port names where possible otherwise rename duplicate components to add a counter at
     * the end.
     * @param symbol The top level component and architecture to flatten.
     * @param map A new HashMap passed for recursion which saves occurrences of components.
     * @return  The top level component with just the computational components as subcomponents.
     */

    public static ExpandedComponentInstanceSymbol flattenArchitecture(ExpandedComponentInstanceSymbol symbol, Map<String,Integer> map){
        if (symbol.getSubComponents().isEmpty()){
            return recursionAnchor(symbol, map);
        }

        for (ExpandedComponentInstanceSymbol sym : symbol.getSubComponents()){
            symbol = flattenArchitecture(sym, map);
        }
        return doAlgorithmStepShortNames(symbol);
    }



    /**
     * Put all computational components in the top level component and preserve all connectors between these components.
     * Rename all components and ports to their full names representing the component structure from the top level
     * component to the component itself.
     * @param symbol The top level component and architecture to flatten.
     * @return  The top level component with just the computational components as subcomponents.
     */

    public static ExpandedComponentInstanceSymbol flattenArchitecture(ExpandedComponentInstanceSymbol symbol){
        if (symbol.getSubComponents().isEmpty()){
            if (symbol.getEnclosingComponent().isPresent()){
                return copySymbolWithSystemName(symbol);
            }
            return symbol;
        }
        for (ExpandedComponentInstanceSymbol sym : symbol.getSubComponents()){
            symbol = flattenArchitecture(sym);
        }

        if (symbol.getEnclosingComponent().isPresent()){
            ExpandedComponentInstanceSymbol enclosingComponent = copySymbolWithSystemName(symbol);
            symbol = enclosingComponent.getSubComponent(symbol.getFullName().replace(".", "_")).get();
            ExpandedComponentInstanceSymbol thisSymbol = symbol;

            List<ExpandedComponentInstanceSymbol> newSubcomponents = getNewSubcomponents(symbol, enclosingComponent);

            HashSet<String> incomingPorts = new HashSet<>(symbol.getIncomingPorts().stream().map(p ->{
                return p.getFullName();
            }).collect(Collectors.toList()));
            HashSet<String> outgoingPorts = new HashSet<>(symbol.getOutgoingPorts().stream().map(p ->{
                return p.getFullName();
            }).collect(Collectors.toList()));

            //only connectors from incoming ports
            Set<ConnectorSymbol> incomingConnectors = symbol.getConnectors().stream()
                    .filter(c -> incomingPorts.contains(thisSymbol.getFullName() + "." + thisSymbol.getName() + "_"+ c.getSource()))
                    .collect(Collectors.toSet());

            //only connectors from outgoing ports
            Set<ConnectorSymbol> outgoingConnectors = symbol.getConnectors().stream()
                    .filter(c -> outgoingPorts.contains(thisSymbol.getFullName() + "." + thisSymbol.getName() + "_"+ c.getTarget()))
                    .collect(Collectors.toSet());

            return connectNewConnectors(symbol, enclosingComponent, newSubcomponents, incomingConnectors, outgoingConnectors);
        } else {
            return symbol;
        }
    }

    /**
     * Put all component from a specific level in the top level component and preserve their connectors.
     * Try to preserve component and port names where possible otherwise rename duplicate components to add a counter at
     * the end.
     * @param symbol The top level component and architecture to flatten.
     * @param map A new HashMap passed for recursion which saves occurrences of components.
     * @param level The amount of levels to go deeper before starting to flatten.
     * @return  The top level component with just the computational components as subcomponents.
     */
    public static ExpandedComponentInstanceSymbol flattenArchitecture(ExpandedComponentInstanceSymbol symbol,
                                                                      Map<String,Integer> map, Integer level){
        if (symbol.getSubComponents().isEmpty() || level == 0){
            return recursionAnchor(symbol, map);
        }

        for (ExpandedComponentInstanceSymbol sym : symbol.getSubComponents()){
            symbol = flattenArchitecture(sym, map, level - 1);
        }
        return doAlgorithmStepShortNames(symbol);
    }

    private static ExpandedComponentInstanceSymbol doAlgorithmStepShortNames(ExpandedComponentInstanceSymbol symbol) {
        if (symbol.getEnclosingComponent().isPresent()) {
            ExpandedComponentInstanceSymbol enclosingComponent = copySymbolWithSystemName(symbol, symbol.getName(), false);
            List<ExpandedComponentInstanceSymbol> newSubcomponents = getNewSubcomponents(symbol, enclosingComponent);

            HashSet<String> incomingPorts = new HashSet<>(symbol.getIncomingPorts().stream().map(p -> {
                return p.getName();
            }).collect(Collectors.toList()));
            HashSet<String> outgoingPorts = new HashSet<>(symbol.getOutgoingPorts().stream().map(p -> {
                return p.getName();
            }).collect(Collectors.toList()));

            //only connectors from incoming ports
            Set<ConnectorSymbol> incomingConnectors = symbol.getConnectors().stream()
                    .filter(c -> incomingPorts.contains(c.getSource()))
                    .collect(Collectors.toSet());

            //only connectors from outgoing ports
            Set<ConnectorSymbol> outgoingConnectors = symbol.getConnectors().stream()
                    .filter(c -> outgoingPorts.contains(c.getTarget()))
                    .collect(Collectors.toSet());
            return connectNewConnectors(symbol, enclosingComponent, newSubcomponents, incomingConnectors, outgoingConnectors);

        } else {
            return symbol;
        }
    }

    private static List<ExpandedComponentInstanceSymbol> getNewSubcomponents(ExpandedComponentInstanceSymbol symbol, ExpandedComponentInstanceSymbol enclosingComponent) {
        List<ExpandedComponentInstanceSymbol> newSubcomponents = enclosingComponent.getSubComponents().stream()
                .filter(e -> !e.getFullName().equals(symbol.getFullName())).collect(Collectors.toList());
        newSubcomponents.addAll(newSubcomponents.size(), new ArrayList<>(symbol.getSubComponents()));
        return newSubcomponents;
    }

    private static ExpandedComponentInstanceSymbol recursionAnchor(ExpandedComponentInstanceSymbol symbol, Map<String, Integer> map) {
        if (symbol.getEnclosingComponent().isPresent()) {
            if (map.containsKey(symbol.getName())) {
                map.replace(symbol.getName(), map.get(symbol.getName()) + 1);
                return copySymbolWithSystemName(symbol, symbol.getName() + map.get(symbol.getName()), true);
            } else {
                map.put(symbol.getName(), 0);
                return symbol.getEnclosingComponent().get();
            }
        }
        return symbol;
    }

    private static ExpandedComponentInstanceSymbol connectNewConnectors(ExpandedComponentInstanceSymbol symbol,
                                                                        ExpandedComponentInstanceSymbol enclosingComponent,
                                                                        List<ExpandedComponentInstanceSymbol> newSubcomponents,
                                                                        Set<ConnectorSymbol> incomingConnectors,
                                                                        Set<ConnectorSymbol> outgoingConnectors) {
        //only connectors going into symbol
        Set<ConnectorSymbol> incomingParentConnectors = enclosingComponent.getConnectors().stream()
                .filter(c -> c.getTargetPort().getComponentInstance().get().getFullName().equals(symbol.getFullName()))
                .collect(Collectors.toSet());

        //only connectors going out of symbol
        Set<ConnectorSymbol> outgoingParentConnectors = enclosingComponent.getConnectors().stream()
                .filter(c -> c.getSourcePort().getComponentInstance().get().getFullName().equals(symbol.getFullName()))
                .collect(Collectors.toSet());

        //untouched connectors of enclosing symbol
        Set<ConnectorSymbol> newConnectors = enclosingComponent.getConnectors().stream()
                .filter(c -> !(incomingParentConnectors.contains(c) || outgoingParentConnectors.contains(c)))
                .collect(Collectors.toSet());

        //untouched connectors of symbol with renamed ports
        newConnectors.addAll(symbol.getConnectors().stream()
                //.map(c -> {return mapToNewName(c);})
                .filter(c -> !(incomingConnectors.contains(c) || outgoingConnectors.contains(c)))
                .collect(Collectors.toSet()));

        connectNewConnectorsIncoming(incomingConnectors, incomingParentConnectors, newConnectors);

        connectNewConnectorsOutgoing(outgoingConnectors, outgoingParentConnectors, newConnectors);

        ExpandedComponentInstanceSymbol res = constructECIS(enclosingComponent, newSubcomponents, newConnectors,
                enclosingComponent.getName(), new ArrayList<>(enclosingComponent.getPortsList()));

        return res;
    }

    private static void connectNewConnectorsOutgoing(Set<ConnectorSymbol> outgoingConnectors,
                                                     Set<ConnectorSymbol> outgoingParentConnectors,
                                                     Set<ConnectorSymbol> newConnectors) {
        for (ConnectorSymbol con : outgoingConnectors) {
            for (ConnectorSymbol connectorSymbol : outgoingParentConnectors) {
                if (con.getTarget().equals(connectorSymbol.getSource().replaceFirst(".*_", ""))) {
                    ConnectorSymbol tmpConnector = ConnectorSymbol.builder()
                            .setSource(con.getSource())
                            .setTarget(connectorSymbol.getTarget())
                            .build();
                    newConnectors.add(tmpConnector);
                }
            }
        }
    }

    private static void connectNewConnectorsIncoming(Set<ConnectorSymbol> incomingConnectors,
                                                     Set<ConnectorSymbol> incomingParentConnectors,
                                                     Set<ConnectorSymbol> newConnectors) {
        for (ConnectorSymbol con : incomingConnectors) {
            for (ConnectorSymbol connectorSymbol : incomingParentConnectors) {
                if (con.getSource().equals(connectorSymbol.getTarget().replaceFirst(".*_", ""))) {
                    ConnectorSymbol tmpConnector = ConnectorSymbol.builder()
                            .setSource(connectorSymbol.getSource())
                            .setTarget(con.getTarget())
                            .build();
                    newConnectors.add(tmpConnector);
                }
            }
        }
    }

    private static ExpandedComponentInstanceSymbol copySymbolWithSystemName(ExpandedComponentInstanceSymbol symbol,
                                                                            String newName, boolean atomic) {
        ExpandedComponentInstanceSymbol enclosingComponent = symbol.getEnclosingComponent().get();
        ExpandedComponentInstanceSymbol thisSymbol = symbol;
        List<ExpandedComponentInstanceSymbol> subcomponents = enclosingComponent.getSubComponents().stream()
                .filter(e -> !e.getFullName().equals(thisSymbol.getFullName())).collect(Collectors.toList());
        List<PortSymbol> ports = new LinkedList<>();
        if (atomic) {
            ports.addAll(symbol.getPortsList());
        }else {
            createNewPorts(symbol, newName, ports);
        }
        ExpandedComponentInstanceSymbol e = constructECIS(symbol, new ArrayList<>(symbol.getSubComponents()),
                new HashSet<>(symbol.getConnectors()), newName, ports);
        subcomponents.add(e);
        HashSet<String> incomingPorts = new HashSet<>(symbol.getIncomingPorts().stream().map(p ->{
            return symbol.getName() + "." + p.getName();
        }).collect(Collectors.toList()));
        HashSet<String> outgoingPorts = new HashSet<>(symbol.getOutgoingPorts().stream().map(p ->{
            return symbol.getName() + "." + p.getName();
        }).collect(Collectors.toList()));
        Set<ConnectorSymbol> newConnectors = enclosingComponent.getConnectors().stream()
                .map(c ->{
                    if (incomingPorts.contains(c.getTarget().substring(0,1).toLowerCase()
                            + c.getTarget().substring(1))){
                        c.setSource(c.getSource());
                        if (atomic){
                            c.setTarget(c.getTarget().replaceFirst("[^.]*.", newName + "."));
                        }else {
                            c.setTarget(c.getTarget().replaceFirst("[^.]*.", newName + "." + newName + "_"));
                        }
                    } else if (outgoingPorts.contains(c.getSource().substring(0,1).toLowerCase()
                            + c.getSource().substring(1))){
                        if (atomic) {
                            c.setSource(c.getSource().replaceFirst("[^.]*.", newName + "."));
                        }else {
                            c.setSource(c.getSource().replaceFirst("[^.]*.",newName + "." + newName + "_"));
                        }
                        c.setTarget(c.getTarget());
                    }
                    return c;
                })
                .collect(Collectors.toSet());
        return constructECIS(enclosingComponent, subcomponents, newConnectors, enclosingComponent.getName(),
                new ArrayList<>(enclosingComponent.getPortsList()));
    }

    private static void createNewPorts(ExpandedComponentInstanceSymbol symbol, String newName, List<PortSymbol> ports) {
        for (PortSymbol port : symbol.getPortsList()) {
            ports.add((PortSymbol) (port.isConstant() ?
                    (new EMAPortBuilder()).setName(newName + "_" + port.getName()).setDirection(port.isIncoming())
                            .setTypeReference(port.getTypeReference()).setConstantValue(((ConstantPortSymbol) port).getConstantValue())
                            .setASTNode(port.getAstNode()).buildConstantPort()
                    : (new EMAPortBuilder()).setName(newName + "_" + port.getName()).setDirection(port.isIncoming())
                    .setTypeReference(port.getTypeReference()).setASTNode(port.getAstNode()).setConfig(port.isConfig())
                    .setMiddlewareSymbol(port.getMiddlewareSymbol()).build()));
        }
    }

    private static ExpandedComponentInstanceSymbol copySymbolWithSystemName(ExpandedComponentInstanceSymbol symbol) {
        ExpandedComponentInstanceSymbol enclosingComponent = symbol.getEnclosingComponent().get();
        ExpandedComponentInstanceSymbol thisSymbol = symbol;
        List<ExpandedComponentInstanceSymbol> subcomponents = enclosingComponent.getSubComponents().stream()
                .filter(e -> !e.getFullName().equals(thisSymbol.getFullName())).collect(Collectors.toList());
        List<PortSymbol> ports = new ArrayList<>();
        String newName = symbol.getFullName().replace(".", "_");
        String newEnclosingName = enclosingComponent.getFullName().replace(".", "_");
        createNewPorts(symbol, newName, ports);
        ExpandedComponentInstanceSymbol e = constructECIS(symbol, new ArrayList<>(symbol.getSubComponents()),
                new HashSet<>(symbol.getConnectors()), newName, ports);
        subcomponents.add(e);
        HashSet<String> incomingPorts = new HashSet<>(symbol.getIncomingPorts().stream().map(p ->{
            return p.getFullName();
        }).collect(Collectors.toList()));
        HashSet<String> outgoingPorts = new HashSet<>(symbol.getOutgoingPorts().stream().map(p ->{
            return p.getFullName();
        }).collect(Collectors.toList()));
        Set<ConnectorSymbol> newConnectors = enclosingComponent.getConnectors().stream()
                .map(c ->{
                    if (incomingPorts.contains(c.getComponentInstance().get().getFullName() + "." +
                            c.getTarget().substring(0,1).toLowerCase() + c.getTarget().substring(1))){
                        c.setSource(c.getSource());
                        c.setTarget(c.getTarget().replaceFirst("[^.]*.",e.getName() + "." + newName + "_"));
                    } else if (outgoingPorts.contains(c.getComponentInstance().get().getFullName() + "." +
                            c.getSource().substring(0,1).toLowerCase() + c.getSource().substring(1))){
                        c.setSource(c.getSource().replaceFirst("[^.]*.",e.getName() + "." + newName + "_"));
                        c.setTarget(c.getTarget());
                    }
                    return c;
                })
                .collect(Collectors.toSet());
        return constructECIS(enclosingComponent, subcomponents, newConnectors, enclosingComponent.getName(),
                new ArrayList<>(enclosingComponent.getPortsList()));
    }

    private static ExpandedComponentInstanceSymbol constructECIS(ExpandedComponentInstanceSymbol enclosingComponent,
                                                                 List<ExpandedComponentInstanceSymbol> newSubcomponents,
                                                                 Set<ConnectorSymbol> newConnectors, String name,
                                                                 List<PortSymbol> ports) {
        Set<ResolvingFilter<? extends Symbol>> resolvingFilters = enclosingComponent.getSpannedScope().getResolvingFilters();

        newSubcomponents.forEach(sc -> ((CommonScope) sc.getSpannedScope()).setResolvingFilters(resolvingFilters));
        ExpandedComponentInstanceSymbol res = new ExpandedComponentInstanceBuilder()
                .setName(name)
                .setSymbolReference(enclosingComponent.getComponentType())
                .addPorts(ports)
                .addConnectors(newConnectors)
                .addSubComponents(newSubcomponents)
                .addResolutionDeclarationSymbols(enclosingComponent.getResolutionDeclarationSymbols())
                .build();

        ((CommonScope) res.getSpannedScope()).setResolvingFilters(resolvingFilters);
        res.setEnclosingScope((MutableScope) enclosingComponent.getEnclosingScope());
        return res;
    }
}
