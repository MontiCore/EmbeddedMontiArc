package de.monticore.lang.monticar.clustering.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.*;

public class MonteCarloRandomStrategy implements MonteCarloStrategy {
    private Random random = new Random();

    public void setRandom(Random random) {
        this.random = random;
    }

    @Override
    public List<Set<EMAComponentInstanceSymbol>> randomClustering(EMAComponentInstanceSymbol componentSymbol, int numberOfClusters) {
        List<Set<EMAComponentInstanceSymbol>> clusters = new ArrayList<>();

        for (int i = 0; i < numberOfClusters; i++) {
            clusters.add(new HashSet<>());
        }

        // All subcomponents of the Symbol
        Collection<EMAComponentInstanceSymbol> subcomponents = componentSymbol.getSubComponents();

        // Put subcomponents into an ArrayList
        ArrayList<EMAComponentInstanceSymbol> arrayListSubComponent = new ArrayList<>();
        for (EMAComponentInstanceSymbol subcomp : subcomponents) {
            arrayListSubComponent.add(subcomp);
        }

        // Distribute randomly!
        // First of all, give each of the clusters one element randomly
        for (int j = 0; j < clusters.size(); j++) {
            if (!arrayListSubComponent.isEmpty()) {
                int randN = randomNumberInRange(0, arrayListSubComponent.size() - 1);
                clusters.get(j).add(arrayListSubComponent.get(randN));
                arrayListSubComponent.remove(randN);
            }
        }

        int numberOfSubcomponents = arrayListSubComponent.size();
        // Then, a random element is assigned to a random cluster until every element is assigned
        for (int h = 0; h < numberOfSubcomponents; h++) {
            int randElement = randomNumberInRange(0, arrayListSubComponent.size() - 1);
            int randCluster = randomNumberInRange(0, clusters.size() - 1);

            clusters.get(randCluster).add(arrayListSubComponent.get(randElement));
            arrayListSubComponent.remove(randElement);
        }
        return clusters;
    }

    public int randomNumberInRange(int min, int max) {
        return random.nextInt((max - min) + 1) + min;
    }
}
