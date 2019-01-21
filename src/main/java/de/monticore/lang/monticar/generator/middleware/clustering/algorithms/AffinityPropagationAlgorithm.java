package de.monticore.lang.monticar.generator.middleware.clustering.algorithms;

import com.clust4j.algo.AffinityPropagation;
import com.clust4j.algo.AffinityPropagationParameters;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.helpers.ComponentHelper;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.*;

public class AffinityPropagationAlgorithm implements ClusteringAlgorithm {

    @Override
    public List<Set<EMAComponentInstanceSymbol>> cluster(EMAComponentInstanceSymbol component, Object... args) {

        List<EMAComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(component);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(component),
                labelsForSubcomps);

        RealMatrix mat = new Array2DRowRealMatrix(adjMatrix);

        AffinityPropagation clustering = new AffinityPropagationParameters().fitNewModel(mat);
        final int[] labels = clustering.getLabels();


        List<Set<EMAComponentInstanceSymbol>> res = new ArrayList<>();

        for(int i = 0; i < labels.length; i++){
            res.add(new HashSet<>());
        }

        subcompsOrderedByName.forEach(sc -> {
            int curClusterLabel = labels[labelsForSubcomps.get(sc.getFullName())];
            res.get(curClusterLabel).add(sc);
        });

        return res;
    }
}
