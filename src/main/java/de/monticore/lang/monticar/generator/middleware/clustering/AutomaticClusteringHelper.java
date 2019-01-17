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



}
