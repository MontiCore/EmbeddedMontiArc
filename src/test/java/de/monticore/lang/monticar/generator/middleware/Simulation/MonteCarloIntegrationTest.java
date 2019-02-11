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

import static de.monticore.lang.monticar.generator.middleware.Simulation.MonteCarloIntegration.calculateCostOfClusters;
import static org.junit.Assert.*;

public class MonteCarloIntegrationTest {

    public static final String TEST_PATH = "src/test/resources/";

    @Test
    public void costTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);

        SpectralClusteringAlgorithm spectralClusteringAlgorithm = new SpectralClusteringAlgorithm();
        Object[] params = new Object[]{SpectralClusteringBuilder.SpectralParameters.SPECTRAL_NUM_CLUSTERS, 3};
        List<Set<EMAComponentInstanceSymbol>> spectralClusters = spectralClusteringAlgorithm.cluster(flattenedComponent, params);
        AutomaticClusteringHelper.annotateComponentWithRosTagsForClusters(flattenedComponent, spectralClusters);

        double cost = calculateCostOfClusters(componentInstanceSymbol, spectralClusters);
        System.out.println("Size of Nodes "+ componentInstanceSymbol.getSubComponents().size());
        System.out.println("Cost: "+ cost);

        assertTrue(cost == 10);
    }

    @Test
    public void mcSimulationTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        double cost = MonteCarloIntegration.simulate(10, componentInstanceSymbol);
        assertTrue(cost == 10);
    }
}