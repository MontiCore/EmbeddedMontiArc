package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class AutomaticClusteringHelper {
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

}
