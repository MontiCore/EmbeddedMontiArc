package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collector;
import java.util.stream.Collectors;

public class AutomaticClusteringHelper {

    static double MAXCOST= 999999;

    public static double[][] createAdjacencyMatrix(List<ExpandedComponentInstanceSymbol> subcomps, Collection<ConnectorSymbol> connectors, Map<String, Integer> subcompLabels) {
        // Nodes = subcomponents
        // Verts = connectors between subcomponents

        double[][] res = new double[subcomps.size()][subcomps.size()];

        connectors.forEach(con -> {
            Optional<ExpandedComponentInstanceSymbol> sourceCompOpt = con.getSourcePort().getComponentInstance();
            Optional<ExpandedComponentInstanceSymbol> targetCompOpt = con.getTargetPort().getComponentInstance();

            if (sourceCompOpt.isPresent() && targetCompOpt.isPresent()) {
                int index1 = subcompLabels.get(sourceCompOpt.get().getFullName());
                int index2 = subcompLabels.get(targetCompOpt.get().getFullName());

                res[index1][index2] = getTypeCostHeuristic(con.getSourcePort());
                res[index2][index1] = getTypeCostHeuristic(con.getSourcePort());
            } else {
                Log.error("0xADE65: Component of source or target not found!");
            }
        });


        return res;
    }

    public static double[][] adjacencyMatrix2transitionMatrix(double[][] adjacencyMatrix) {
        double[][] transitionMatrix= adjacencyMatrix;

        int degree;
        for(int i = 0; i < adjacencyMatrix[0].length; i++) {
            degree= 0;
            for(int j = 0; j < adjacencyMatrix[0].length; j++) {
                if (adjacencyMatrix[i][j] == 1) degree++;
            }
            for(int j = 0; j < adjacencyMatrix[0].length; j++) {
                if (adjacencyMatrix[i][j] == 1) transitionMatrix[i][j] = 1.0/degree;
            }
        }

        return transitionMatrix;
    }


    // generic matrix normalizer
    public static double[][] normalizeMatrix(double[][] matrix) {
        double[][] normalizedMatrix= matrix;

        double normalizer;
        double sum;
        for(int i = 0; i < matrix[0].length; i++) {
            normalizer= 0;
            sum= 0;
            for(int j = 0; j < matrix[0].length; j++) {
                sum+= normalizedMatrix[i][j];
            }
            if (sum>0) normalizer= 1.0/sum;
            for(int j = 0; j < matrix[0].length; j++) {
                normalizedMatrix[i][j] = matrix[i][j] * normalizer;
            }
        }

        return normalizedMatrix;
    }

    // calculate the inverse probabilities of a transition matrix
    // (regard zero as immutable zero probability)
    public static double[][] inverseProbabilitiesMatrix(double[][] matrix) {
        double[][] inverseProbabilityMatrix= matrix;

        for(int i = 0; i < matrix[0].length; i++) {
            for (int j = 0; j < matrix[0].length; j++) {
                if (matrix[i][j] > 0) inverseProbabilityMatrix[i][j] = 1.0/matrix[i][j];
            }
        }

        return inverseProbabilityMatrix;
    }

    // Weights in the adjacency matrix are regarded cost or penalty.
    // They are seen as "inverse probability" (1/prob) in the transition matrix.
    public static double[][] weightedAdjacencyMatrix2transitionMatrix(double[][] adjacencyMatrix) {
        return normalizeMatrix(inverseProbabilitiesMatrix(adjacencyMatrix));
    }

    public static void annotateComponentWithRosTagsForClusters(ExpandedComponentInstanceSymbol componentInstanceSymbol, List<Set<ExpandedComponentInstanceSymbol>> clusters) {
        Collection<ConnectorSymbol> connectors = componentInstanceSymbol.getConnectors();

        connectors.forEach(con -> {
            // -1 = super comp
            int sourceClusterLabel = -1;
            int targetClusterLabel = -1;

            ExpandedComponentInstanceSymbol sourceComp = con.getSourcePort().getComponentInstance().get();
            ExpandedComponentInstanceSymbol targetComp = con.getTargetPort().getComponentInstance().get();

            for(int i = 0; i < clusters.size(); i++){
                if(clusters.get(i).contains(sourceComp)){
                    sourceClusterLabel = i;
                }

                if(clusters.get(i).contains(targetComp)){
                    targetClusterLabel = i;
                }
            }

            if(sourceClusterLabel != targetClusterLabel){
                con.getSourcePort().setMiddlewareSymbol(new RosConnectionSymbol());
                con.getTargetPort().setMiddlewareSymbol(new RosConnectionSymbol());
            }

        });

    }

    public static double getTypeCostHeuristic(PortSymbol port){
        return getTypeCostHeuristic(port.getTypeReference());
    }

    public static double getTypeCostHeuristic(MCTypeReference<? extends MCTypeSymbol> typeReference) {
        if (typeReference.getName().equals("CommonMatrixType")){
            double value = getTypeCostHeuristicHelper(
                    ((ASTCommonMatrixType)((MCASTTypeSymbolReference)typeReference).getAstType()).getElementType().getName());
            double res = 0;
            List<ASTExpression> vectors = ((ASTCommonMatrixType) ((MCASTTypeSymbolReference) typeReference).
                    getAstType()).getDimension().getDimensionList();
            for (ASTExpression expression : vectors){
                if (((ASTNumberExpression) expression).getNumberWithUnit().getNumber().isPresent()) {
                    res += value * ((ASTNumberExpression) expression).getNumberWithUnit().getNumber().get();
                }
            }
            return res;
        } else {
            return getTypeCostHeuristicHelper(typeReference.getName());
        }
    }

    private static double getTypeCostHeuristicHelper(String name) {
        double bool = 1;
        double z = 5;
        double q = 10;
        double c = 20;
        switch (name) {
            case "B":
                return bool;
            case "Z":
                return z;
            case "Q":
                return q;
            case "C":
                return c;
        }
        return 50;
    }

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

            List<ExpandedComponentInstanceSymbol> newSubcomponents = enclosingComponent.getSubComponents().stream()
                    .filter(e -> !e.getFullName().equals(thisSymbol.getFullName())).collect(Collectors.toList());
            newSubcomponents.addAll(newSubcomponents.size(), new ArrayList<>(symbol.getSubComponents()));

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

            //only connectors going into symbol
            Set<ConnectorSymbol> incomingParentConnectors = enclosingComponent.getConnectors().stream()
                    .filter(c -> c.getTargetPort().getComponentInstance().get().getFullName().equals(thisSymbol.getFullName()))
                    .collect(Collectors.toSet());

            //only connectors from outgoing ports
            Set<ConnectorSymbol> outgoingConnectors = symbol.getConnectors().stream()
                    .filter(c -> outgoingPorts.contains(thisSymbol.getFullName() + "." + thisSymbol.getName() + "_"+ c.getTarget()))
                    .collect(Collectors.toSet());

            //only connectors going out of symbol
            Set<ConnectorSymbol> outgoingParentConnectors = enclosingComponent.getConnectors().stream()
                    .filter(c -> c.getSourcePort().getComponentInstance().get().getFullName().equals(thisSymbol.getFullName()))
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

            for (ConnectorSymbol con : incomingConnectors){
                for (ConnectorSymbol connectorSymbol : incomingParentConnectors){
                    if (con.getSource().equals(connectorSymbol.getTarget().replaceFirst(".*_", ""))){
                        ConnectorSymbol tmpConnector = ConnectorSymbol.builder()
                                .setSource(connectorSymbol.getSource())
                                .setTarget(con.getTarget())
                                .build();
                        newConnectors.add(tmpConnector);
                    }
                }
            }

            for (ConnectorSymbol con : outgoingConnectors){
                for (ConnectorSymbol connectorSymbol : outgoingParentConnectors){
                    if (con.getTarget().equals(connectorSymbol.getSource().replaceFirst(".*_", ""))){
                        ConnectorSymbol tmpConnector = ConnectorSymbol.builder()
                                .setSource(con.getSource())
                                .setTarget(connectorSymbol.getTarget())
                                .build();
                        newConnectors.add(tmpConnector);
                    }
                }
            }

            ExpandedComponentInstanceSymbol res = constructECIS(enclosingComponent, newSubcomponents, newConnectors,
                    enclosingComponent.getName(), new ArrayList<>(enclosingComponent.getPortsList()));

            return res;
        } else {
            return symbol;
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
        for (PortSymbol port : symbol.getPortsList()) {
            ports.add((PortSymbol)(port.isConstant() ?
                    (new EMAPortBuilder()).setName(newName + "_" + port.getName()).setDirection(port.isIncoming())
                            .setTypeReference(port.getTypeReference()).setConstantValue(((ConstantPortSymbol)port).getConstantValue())
                            .setASTNode(port.getAstNode()).buildConstantPort()
                    : (new EMAPortBuilder()).setName(newName + "_" + port.getName()).setDirection(port.isIncoming())
                    .setTypeReference(port.getTypeReference()).setASTNode(port.getAstNode()).setConfig(port.isConfig())
                    .setMiddlewareSymbol(port.getMiddlewareSymbol()).build()));
        }
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
                    if (incomingPorts.contains(c.getComponentInstance().get().getFullName() + "." + c.getTarget().substring(0,1).toLowerCase()
                            + c.getTarget().substring(1))){
                        c.setSource(c.getSource());
                        c.setTarget(c.getTarget().replaceFirst("[^.]*.",e.getName() + "." + newName + "_"));
                    } else if (outgoingPorts.contains(c.getComponentInstance().get().getFullName() + "." + c.getSource().substring(0,1).toLowerCase()
                            + c.getSource().substring(1))){
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
                                                                 Set<ConnectorSymbol> newConnectors, String name, List<PortSymbol> ports) {
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
