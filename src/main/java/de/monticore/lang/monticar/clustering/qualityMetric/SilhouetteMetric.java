/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.qualityMetric;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.ClusteringInput;
import de.monticore.lang.monticar.clustering.ClusteringResult;

import java.util.List;
import java.util.Map;
import java.util.Set;

public class SilhouetteMetric implements Metric {
    @Override
    public boolean higherIsBetter() {
        return true;
    }

    @Override
    public double getScore(ClusteringResult clusteringResult) {
        ClusteringInput clusteringInput = clusteringResult.getClusteringInput();
        int[] labels = getClusteringLabels(clusteringInput, clusteringResult.getClustering());
        return new SilhouetteIndex(clusteringInput.getDistanceMatrix(), labels).getSilhouetteScore();
    }

    private int[] getClusteringLabels(ClusteringInput clusteringInput, List<Set<EMAComponentInstanceSymbol>> clustering) {
        int[] labels = new int[clusteringInput.getSubcompsOrderedByName().size()];
        Map<String, Integer> subcompLabels = clusteringInput.getLabelsForSubcomps();

        for (int i = 0; i < clustering.size(); i++) {
            for (EMAComponentInstanceSymbol emaComponentInstanceSymbol : clustering.get(i)) {
                labels[subcompLabels.get(emaComponentInstanceSymbol.getFullName())] = i;
            }

        }
        return labels;
    }

}
