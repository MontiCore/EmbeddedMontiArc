/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.ClusteringInput;
import de.se_rwth.commons.logging.Log;

import java.util.*;

/**
 * A Monte Carlo clustering strategy based on Karger's algorithm
 */
public class MonteCarloKargerStrategy implements MonteCarloStrategy{
    private Random random = new Random();
    private boolean warned = false;

    public void setRandom(Random random) {
        this.random = random;
    }

    @Override
    public List<Set<EMAComponentInstanceSymbol>> randomClustering(ClusteringInput input, int numberOfClusters) {
        List<EMAComponentInstanceSymbol> subcomps = input.getSubcompsOrderedByName();
        if(subcomps.size() < numberOfClusters){
            Log.error("The component has less subcomponents then clusters requested!");
            return new ArrayList<>();
        }
        Map<Integer, Set<Integer>> labelToSet = new HashMap<>();

        // Get representative connectors(^= edges)
        List<Edge<Integer>> edges = input.getUniqueLabeledEdges();

        List<Set<Integer>> buffer = new ArrayList<>();
        for(EMAComponentInstanceSymbol rep : subcomps){
            buffer.add(new HashSet<>());
        }

        for(int i = 0; i < subcomps.size(); i++){
            Set<Integer> curSet = buffer.get(i);
            labelToSet.put(i, curSet);
            curSet.add(i);
        }

        while(buffer.size() > numberOfClusters){
            if(edges.size() == 0){
                if(!warned) {
                    Log.warn("No more edges to collapse but more clusters then requested are left! Randomly mergin clusters from now onward.");
                    warned = true;
                }
                Set<Integer> firstCluster = popRandomFromList(buffer);
                Set<Integer> secondCluster = popRandomFromList(buffer);
                for (Integer label : secondCluster) {
                    labelToSet.put(label, firstCluster);
                    firstCluster.add(label);
                }
                buffer.add(firstCluster);
            }else {
                Edge<Integer> curEdge = popRandomFromList(edges);
                Set<Integer> firstCluster = labelToSet.get(curEdge.getFirst());
                Set<Integer> secondCluster = labelToSet.get(curEdge.getSecond());
                if(firstCluster != secondCluster) {
                    buffer.remove(secondCluster);
                    for (Integer label : secondCluster) {
                        labelToSet.put(label, firstCluster);
                        firstCluster.add(label);
                    }
                }
            }
        }

        Map<String, Integer> compsToLabels = input.getLabelsForSubcomps();

        List<Set<EMAComponentInstanceSymbol>> res = new ArrayList<>();
        for (Set<Integer> labelSet : buffer) {
            HashSet<EMAComponentInstanceSymbol> resSet = new HashSet<>();
            for (Integer label : labelSet) {
                resSet.add(input.getSubcompForLabel(label));
            }
            res.add(resSet);
        }

        res.removeIf(Set::isEmpty);
        return res;
    }

    private <T> T popRandomFromList(List<? extends T> list){
        int index = random.nextInt(list.size());
        return list.remove(index);
    }

    private class SetWithRepresentative<T> extends HashSet<T>{
        private T representative;

        public T getRepresentative() {
            return representative;
        }

        public void setRepresentative(T representative) {
            this.representative = representative;
        }
    }

}
