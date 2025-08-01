/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.CommonSymbol;
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

    public static EMAComponentInstanceSymbol flattenArchitecture(EMAComponentInstanceSymbol symbol, Map<String,Integer> map){
        if (symbol.getSubComponents().isEmpty()){
            return recursionAnchor(symbol, map);
        }

        for (EMAComponentInstanceSymbol sym : symbol.getSubComponents()){
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

    public static EMAComponentInstanceSymbol flattenArchitecture(EMAComponentInstanceSymbol symbol){
        if (symbol.getSubComponents().isEmpty()){
            if (symbol.getEnclosingComponent().isPresent()){
                return copySymbolWithSystemName(symbol);
            }
            return symbol;
        }
        for (EMAComponentInstanceSymbol sym : symbol.getSubComponents()){
            symbol = flattenArchitecture(sym);
        }

        if (symbol.getEnclosingComponent().isPresent()){
            EMAComponentInstanceSymbol enclosingComponent = copySymbolWithSystemName(symbol);
            symbol = enclosingComponent.getSubComponent(symbol.getFullName().replace(".", "_")).get();
            EMAComponentInstanceSymbol thisSymbol = symbol;

            List<EMAComponentInstanceSymbol> newSubcomponents = getNewSubcomponents(symbol, enclosingComponent);

            HashSet<String> incomingPorts = symbol.getIncomingPortInstances().stream().map(CommonSymbol::getFullName).collect(Collectors.toCollection(HashSet::new));
            HashSet<String> outgoingPorts = symbol.getOutgoingPortInstances().stream().map(CommonSymbol::getFullName).collect(Collectors.toCollection(HashSet::new));

            //only connectors from incoming ports
            Set<EMAConnectorSymbol> incomingConnectors = symbol.getConnectorInstances().stream()
                    .filter(c -> incomingPorts.contains(thisSymbol.getFullName() + "." + thisSymbol.getName() + "_"+ c.getSource()))
                    .collect(Collectors.toSet());

            //only connectors from outgoing ports
            Set<EMAConnectorSymbol> outgoingConnectors = symbol.getConnectorInstances().stream()
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
    public static EMAComponentInstanceSymbol flattenArchitecture(EMAComponentInstanceSymbol symbol,
                                                                      Map<String,Integer> map, Integer level){
        if (symbol.getSubComponents().isEmpty() || level == 0){
            return recursionAnchor(symbol, map);
        }

        for (EMAComponentInstanceSymbol sym : symbol.getSubComponents()){
            symbol = flattenArchitecture(sym, map, level - 1);
        }
        return doAlgorithmStepShortNames(symbol);
    }

    private static EMAComponentInstanceSymbol doAlgorithmStepShortNames(EMAComponentInstanceSymbol symbol) {
        if (symbol.getEnclosingComponent().isPresent()) {
            EMAComponentInstanceSymbol enclosingComponent = copySymbolWithSystemName(symbol, symbol.getName(), false);
            List<EMAComponentInstanceSymbol> newSubcomponents = getNewSubcomponents(symbol, enclosingComponent);

            HashSet<String> incomingPorts = symbol.getIncomingPortInstances().stream().map(CommonSymbol::getName).collect(Collectors.toCollection(HashSet::new));
            HashSet<String> outgoingPorts = symbol.getOutgoingPortInstances().stream().map(CommonSymbol::getName).collect(Collectors.toCollection(HashSet::new));

            //only connectors from incoming ports
            Set<EMAConnectorSymbol> incomingConnectors = symbol.getConnectorInstances().stream()
                    .filter(c -> incomingPorts.contains(c.getSource()))
                    .collect(Collectors.toSet());

            //only connectors from outgoing ports
            Set<EMAConnectorSymbol> outgoingConnectors = symbol.getConnectorInstances().stream()
                    .filter(c -> outgoingPorts.contains(c.getTarget()))
                    .collect(Collectors.toSet());
            return connectNewConnectors(symbol, enclosingComponent, newSubcomponents, incomingConnectors, outgoingConnectors);

        } else {
            return symbol;
        }
    }

    private static List<EMAComponentInstanceSymbol> getNewSubcomponents(EMAComponentInstanceSymbol symbol, EMAComponentInstanceSymbol enclosingComponent) {
        List<EMAComponentInstanceSymbol> newSubcomponents = enclosingComponent.getSubComponents().stream()
                .filter(e -> !e.getFullName().equals(symbol.getFullName())).collect(Collectors.toList());
        newSubcomponents.addAll(newSubcomponents.size(), new ArrayList<>(symbol.getSubComponents()));
        return newSubcomponents;
    }

    private static EMAComponentInstanceSymbol recursionAnchor(EMAComponentInstanceSymbol symbol, Map<String, Integer> map) {
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

    private static EMAComponentInstanceSymbol connectNewConnectors(EMAComponentInstanceSymbol symbol,
                                                                        EMAComponentInstanceSymbol enclosingComponent,
                                                                        List<EMAComponentInstanceSymbol> newSubcomponents,
                                                                        Set<EMAConnectorSymbol> incomingConnectors,
                                                                        Set<EMAConnectorSymbol> outgoingConnectors) {
        //only connectors going into symbol
        Set<EMAConnectorSymbol> incomingParentConnectors = enclosingComponent.getConnectorInstances().stream()
                .filter(c -> c.getTargetPort().getComponentInstance().getFullName().equals(symbol.getFullName()))
                .collect(Collectors.toSet());

        //only connectors going out of symbol
        Set<EMAConnectorSymbol> outgoingParentConnectors = enclosingComponent.getConnectorInstances().stream()
                .filter(c -> c.getSourcePort().getComponentInstance().getFullName().equals(symbol.getFullName()))
                .collect(Collectors.toSet());

        //untouched connectors of enclosing symbol
        Set<EMAConnectorSymbol> newConnectors = enclosingComponent.getConnectorInstances().stream()
                .filter(c -> !(incomingParentConnectors.contains(c) || outgoingParentConnectors.contains(c)))
                .collect(Collectors.toSet());

        //untouched connectors of symbol with renamed ports
        newConnectors.addAll(symbol.getConnectorInstances().stream()
                //.map(c -> {return mapToNewName(c);})
                .filter(c -> !(incomingConnectors.contains(c) || outgoingConnectors.contains(c)))
                .collect(Collectors.toSet()));

        connectNewConnectorsIncoming(incomingConnectors, incomingParentConnectors, newConnectors);

        connectNewConnectorsOutgoing(outgoingConnectors, outgoingParentConnectors, newConnectors);

        EMAComponentInstanceSymbol res = constructECIS(enclosingComponent, newSubcomponents, newConnectors,
                enclosingComponent.getName(), new ArrayList<>(enclosingComponent.getPortInstanceList()));

        return res;
    }

    private static void connectNewConnectorsOutgoing(Set<EMAConnectorSymbol> outgoingConnectors,
                                                     Set<EMAConnectorSymbol> outgoingParentConnectors,
                                                     Set<EMAConnectorSymbol> newConnectors) {
        for (EMAConnectorSymbol con : outgoingConnectors) {
            for (EMAConnectorSymbol EMAConnectorInstanceSymbol : outgoingParentConnectors) {
                if (con.getTarget().equals(EMAConnectorInstanceSymbol.getSource().replaceFirst(".*_", ""))) {
                    EMAConnectorSymbol tmpConnector = EMAConnectorInstanceSymbol.builder()
                            .setSource(con.getSource())
                            .setTarget(EMAConnectorInstanceSymbol.getTarget())
                            .build();
                    newConnectors.add(tmpConnector);
                }
            }
        }
    }

    private static void connectNewConnectorsIncoming(Set<EMAConnectorSymbol> incomingConnectors,
                                                     Set<EMAConnectorSymbol> incomingParentConnectors,
                                                     Set<EMAConnectorSymbol> newConnectors) {
        for (EMAConnectorSymbol con : incomingConnectors) {
            for (EMAConnectorSymbol EMAConnectorInstanceSymbol : incomingParentConnectors) {
                if (con.getSource().equals(EMAConnectorInstanceSymbol.getTarget().replaceFirst(".*_", ""))) {
                    EMAConnectorSymbol tmpConnector = EMAConnectorInstanceSymbol.builder()
                            .setSource(EMAConnectorInstanceSymbol.getSource())
                            .setTarget(con.getTarget())
                            .build();
                    newConnectors.add(tmpConnector);
                }
            }
        }
    }

    private static EMAComponentInstanceSymbol copySymbolWithSystemName(EMAComponentInstanceSymbol symbol,
                                                                            String newName, boolean atomic) {
        EMAComponentInstanceSymbol enclosingComponent = symbol.getEnclosingComponent().get();
        EMAComponentInstanceSymbol thisSymbol = symbol;
        List<EMAComponentInstanceSymbol> subcomponents = enclosingComponent.getSubComponents().stream()
                .filter(e -> !e.getFullName().equals(thisSymbol.getFullName())).collect(Collectors.toList());
        List<EMAPortSymbol> ports = new LinkedList<>();
        if (atomic) {
            ports.addAll(symbol.getPortInstanceList());
        }else {
            createNewPorts(symbol, newName, ports);
        }
        EMAComponentInstanceSymbol e = constructECIS(symbol, new ArrayList<>(symbol.getSubComponents()),
                new HashSet<>(symbol.getConnectorInstances()), newName, ports);
        subcomponents.add(e);
        HashSet<String> incomingPorts = symbol.getIncomingPortInstances().stream().map(p -> symbol.getName() + "." + p.getName()).collect(Collectors.toCollection(HashSet::new));
        HashSet<String> outgoingPorts = symbol.getOutgoingPortInstances().stream().map(p -> symbol.getName() + "." + p.getName()).collect(Collectors.toCollection(HashSet::new));
        Set<EMAConnectorSymbol> newConnectors = enclosingComponent.getConnectorInstances().stream()
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
                new ArrayList<>(enclosingComponent.getPortInstanceList()));
    }

    private static void createNewPorts(EMAComponentInstanceSymbol symbol, String newName, List<EMAPortSymbol> ports) {
        for (EMAPortInstanceSymbol port : symbol.getPortInstanceList()) {
            ports.add((port.isConstant() ?
                    (new EMAPortBuilder()).setName(newName + "_" + port.getName()).setDirection(port.isIncoming())
                            .setTypeReference(port.getTypeReference()).setConstantValue(port.getConstantValue().get())
                            .setASTNode(port.getAstNode()).build()
                    : (new EMAPortBuilder()).setName(newName + "_" + port.getName()).setDirection(port.isIncoming())
                    .setTypeReference(port.getTypeReference()).setASTNode(port.getAstNode()).setConfig(port.isConfig())
                    .setMiddlewareSymbol(port.getMiddlewareSymbol().orElse(null)).build()));
        }
    }

    private static EMAComponentInstanceSymbol copySymbolWithSystemName(EMAComponentInstanceSymbol symbol) {
        EMAComponentInstanceSymbol enclosingComponent = symbol.getEnclosingComponent().get();
        EMAComponentInstanceSymbol thisSymbol = symbol;
        List<EMAComponentInstanceSymbol> subcomponents = enclosingComponent.getSubComponents().stream()
                .filter(e -> !e.getFullName().equals(thisSymbol.getFullName())).collect(Collectors.toList());
        List<EMAPortSymbol> ports = new ArrayList<>();
        String newName = symbol.getFullName().replace(".", "_");
        String newEnclosingName = enclosingComponent.getFullName().replace(".", "_");
        createNewPorts(symbol, newName, ports);
        EMAComponentInstanceSymbol e = constructECIS(symbol, new ArrayList<>(symbol.getSubComponents()),
                new HashSet<>(symbol.getConnectorInstances()), newName, ports);
        subcomponents.add(e);
        HashSet<String> incomingPorts = symbol.getIncomingPortInstances().stream().map(CommonSymbol::getFullName).collect(Collectors.toCollection(HashSet::new));
        HashSet<String> outgoingPorts = symbol.getOutgoingPortInstances().stream().map(CommonSymbol::getFullName).collect(Collectors.toCollection(HashSet::new));
        Set<EMAConnectorSymbol> newConnectors = enclosingComponent.getConnectorInstances().stream()
                .map(c ->{
                    if (incomingPorts.contains(c.getComponentInstance().getFullName() + "." +
                            c.getTarget().substring(0,1).toLowerCase() + c.getTarget().substring(1))){
                        c.setSource(c.getSource());
                        c.setTarget(c.getTarget().replaceFirst("[^.]*.",e.getName() + "." + newName + "_"));
                    } else if (outgoingPorts.contains(c.getComponentInstance().getFullName() + "." +
                            c.getSource().substring(0,1).toLowerCase() + c.getSource().substring(1))){
                        c.setSource(c.getSource().replaceFirst("[^.]*.",e.getName() + "." + newName + "_"));
                        c.setTarget(c.getTarget());
                    }
                    return c;
                })
                .collect(Collectors.toSet());
        return constructECIS(enclosingComponent, subcomponents, newConnectors, enclosingComponent.getName(),
                new ArrayList<>(enclosingComponent.getPortInstanceList()));
    }

    private static EMAComponentInstanceSymbol constructECIS(EMAComponentInstanceSymbol enclosingComponent,
                                                                 List<EMAComponentInstanceSymbol> newSubcomponents,
                                                                 Set<EMAConnectorSymbol> newConnectors, String name,
                                                                 List<EMAPortSymbol> ports) {
        Set<ResolvingFilter<? extends Symbol>> resolvingFilters = enclosingComponent.getSpannedScope().getResolvingFilters();

        newSubcomponents.forEach(sc -> {
            ((CommonScope) sc.getSpannedScope()).setResolvingFilters(resolvingFilters);
            sc.setEnclosingScope(null);
        });

        newSubcomponents.forEach(sc -> ((CommonScope) sc.getSpannedScope()).setResolvingFilters(resolvingFilters));
        EMAComponentInstanceSymbol res = new EMAComponentInstanceBuilder()
                .setName(name)
                .setPackageName(enclosingComponent.getPackageName())
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
