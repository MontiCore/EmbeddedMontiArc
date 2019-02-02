package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonSymbol;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class AutomaticClusteringHelper {

    static double MAXCOST= 999999;

    public static double[][] createAdjacencyMatrix(List<EMAComponentInstanceSymbol> subcomps, Collection<EMAConnectorInstanceSymbol> connectors, Map<String, Integer> subcompLabels) {
        // Nodes = subcomponents
        // Verts = connectors between subcomponents

        double[][] res = new double[subcomps.size()][subcomps.size()];

        connectors.forEach(con -> {
            EMAComponentInstanceSymbol sourceCompOpt = con.getSourcePort().getComponentInstance();
            EMAComponentInstanceSymbol targetCompOpt = con.getTargetPort().getComponentInstance();

                int index1 = subcompLabels.get(sourceCompOpt.getFullName());
                int index2 = subcompLabels.get(targetCompOpt.getFullName());

                res[index1][index2] = getTypeCostHeuristic(con.getSourcePort());
                res[index2][index1] = getTypeCostHeuristic(con.getSourcePort());
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

    public static void annotateComponentWithRosTagsForClusters(EMAComponentInstanceSymbol componentInstanceSymbol, List<Set<EMAComponentInstanceSymbol>> clusters) {
        Collection<EMAConnectorInstanceSymbol> connectors = componentInstanceSymbol.getConnectorInstances();

        connectors.forEach(con -> {
            // -1 = super comp
            int sourceClusterLabel = -1;
            int targetClusterLabel = -1;

            EMAComponentInstanceSymbol sourceComp = con.getSourcePort().getComponentInstance();
            EMAComponentInstanceSymbol targetComp = con.getTargetPort().getComponentInstance();

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

    public static double getTypeCostHeuristic(EMAComponentInstanceSymbol componentInstanceSymbol, List<Set<EMAComponentInstanceSymbol>> clustering){
        List<Set<String>> clusteringAsNames = clustering.stream()
                .map(s -> s.stream()
                        .map(CommonSymbol::getFullName)
                        .collect(Collectors.toSet()))
                .collect(Collectors.toList());

        List<EMAConnectorInstanceSymbol> interClusterConnectors = componentInstanceSymbol.getConnectorInstances().stream()
                .filter(con -> {
                    EMAComponentInstanceSymbol sourceComp = con.getSourcePort().getComponentInstance();
                    EMAComponentInstanceSymbol targetComp = con.getTargetPort().getComponentInstance();

                    int sourceClusterIndex = -1;
                    int targetClusterIndex = -1;

                    for (int i = 0; i < clusteringAsNames.size(); i++) {
                        if (clusteringAsNames.get(i).contains(sourceComp.getFullName())) {
                            sourceClusterIndex = i;
                        }
                        if (clusteringAsNames.get(i).contains(targetComp.getFullName())) {
                            targetClusterIndex = i;
                        }
                    }

                    return sourceClusterIndex != targetClusterIndex;
                })
                .collect(Collectors.toList());

        return interClusterConnectors.stream()
                .map(EMAConnectorInstanceSymbol::getTargetPort)
                .map(AutomaticClusteringHelper::getTypeCostHeuristic)
                .mapToDouble(d -> d)
                .sum();
    }

    public static double getTypeCostHeuristic(EMAPortSymbol port){
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


    public static ClusteringResultList executeClusteringFromParams(EMAComponentInstanceSymbol emaComponentInstance, List<AlgorithmCliParameters> algoParams) {
        ClusteringResultList res = new ClusteringResultList();
        for (int i = 0; i < algoParams.size(); i++) {
            System.out.println("Clustering with algorithm " + (i+1) + "/" + algoParams.size() + ": " +algoParams.get(i).toString());
            res.add(ClusteringResult.fromParameters(emaComponentInstance, algoParams.get(i)));
        }
        return res;
    }

}
