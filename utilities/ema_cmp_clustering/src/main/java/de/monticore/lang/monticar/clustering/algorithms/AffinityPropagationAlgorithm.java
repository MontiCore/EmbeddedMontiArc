/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.algorithms;

import com.clust4j.algo.AffinityPropagation;
import com.clust4j.algo.AffinityPropagationParameters;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.clustering.ClusteringInput;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.*;

public class AffinityPropagationAlgorithm implements ClusteringAlgorithm {

    private Object[] args;

    public void setArgs(Object[] args) {
        this.args = args;
    }

    @Override
    public Object[] getArgs() {
        return args;
    }

    @Override
    public List<Set<EMAComponentInstanceSymbol>> cluster(ClusteringInput clusteringInput, Object... args) {

        RealMatrix mat = new Array2DRowRealMatrix(clusteringInput.getAdjacencyMatrix());

        AffinityPropagation clustering;
        if (args != null && args.length == 2 && args[0] == AffinityPropagationBuilder.AffinityPropagationParameters.SEED) {
            clustering = new AffinityPropagationParameters().setSeed(new Random((Long) args[1])).fitNewModel(mat);
        }else{
            clustering = new AffinityPropagationParameters().fitNewModel(mat);
        }

        final int[] labels = clustering.getLabels();


        List<Set<EMAComponentInstanceSymbol>> res = new ArrayList<>();

        for(int i = 0; i < labels.length; i++){
            res.add(new HashSet<>());
        }

        clusteringInput.getSubcompsOrderedByName().forEach(sc -> {
            int curClusterLabel = labels[clusteringInput.getLabelsForSubcomps().get(sc.getFullName())];
            res.get(curClusterLabel).add(sc);
        });

        return res;
    }
}
