package de.monticore.lang.monticar.generator.middleware.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.FlattenArchitecture;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringBuilder;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.List;
import java.util.Set;

import static org.junit.Assert.*;

public class MonteCarloIntegrationTest {

    public static final String TEST_PATH = "src/test/resources/";

    @Test
    public void costRandomClustering(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        System.out.println("Size of Subcomponents: " + componentInstanceSymbol.getSubComponents().size());
        System.out.println("Components: \n" + componentInstanceSymbol);
        //System.out.println(componentInstanceSymbol);
        List<Set<EMAComponentInstanceSymbol>> clusters = MonteCarloIntegration.randomClustering(componentInstanceSymbol, 2);


        System.out.println(" ------------------");
        System.out.println(" ");
        System.out.println("Clusters: \n" + clusters);
        assertTrue(clusters.size() == 2);
        assertTrue(clusters.get(0).size()>=1);
        assertTrue(clusters.get(1).size()>=1);
    }


    @Test
    public void mcSimulationTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        double cost = MonteCarloIntegration.simulate(100, componentInstanceSymbol, 2);
        System.out.println("Cost: "+cost);
        assertTrue(cost >= 10);
    }
}