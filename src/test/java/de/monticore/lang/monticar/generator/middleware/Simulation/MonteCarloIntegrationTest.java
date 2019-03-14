package de.monticore.lang.monticar.generator.middleware.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.middleware.clustering.FlattenArchitecture;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Ignore;
import org.junit.Test;

import java.util.List;
import java.util.Set;

import static org.junit.Assert.*;
import static org.junit.Assert.assertTrue;

public class MonteCarloIntegrationTest {

    public static final String TEST_PATH = "src/test/resources/";
    public static final String TEST_PATH_AUTOPILOT = "src/test/resources/autopilot/";

    @Test
    public void randomClusteringTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);
        List<Set<EMAComponentInstanceSymbol>> clusters = MonteCarloIntegration.randomClustering(flattenedComponent, 2);


        assertTrue("Too many or less clusters created.", clusters.size() == 2);
        assertTrue("Subcomponent 1 is not distributed evenly/correctly!", clusters.get(0).size()>=1);
        assertTrue("Subcomponent 2 is not distributed evenly/correctly!",clusters.get(1).size()>=1);
    }

    @Ignore
    @Test
    public void monteCarloSimulationTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);

        //Random Clustering
        MonteCarloIntegration sim = new MonteCarloIntegration(1000, 3);
        double costRandom = MonteCarloIntegration.simulate(flattenedComponent);
        double[] averages = MonteCarloIntegration.getAverages();
        double[] costs = MonteCarloIntegration.getCosts();

        assertTrue(averages[1]== (costs[0]+costs[1])/2);
        assertTrue(averages[2]== (costs[0]+costs[1]+costs[2])/3);
        assertTrue(averages[3]== (costs[0]+costs[1]+costs[2]+costs[3])/4);
    }

    @Ignore
    @Test
    public void generateJSONFileAutopilot(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("de.rwth.armin.modeling.autopilot.autopilot", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);

        //Random Clustering
        MonteCarloIntegration sim = new MonteCarloIntegration(1000, 3);
        double costRandom = MonteCarloIntegration.simulate(flattenedComponent);

        MonteCarloResult res = new MonteCarloResult(flattenedComponent, 1000, 3);

        res.saveAsJson("target/evaluation/montecarlo", "autopilot.json");

    }


}