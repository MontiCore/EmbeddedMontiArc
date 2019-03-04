package de.monticore.lang.monticar.generator.middleware.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.middleware.clustering.FlattenArchitecture;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.List;
import java.util.Set;

import static org.junit.Assert.*;

public class MonteCarloIntegrationTest {

    public static final String TEST_PATH = "src/test/resources/";
    public static final String TEST_PATH_PACMAN = "src/test/resources/pacman/";

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

    @Test
    public void monteCarloSimulationTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);

        // random clustering
        double cost = MonteCarloIntegration.simulate(100, flattenedComponent, 2, 2);

        // spectral clustering
        double cost2 = MonteCarloIntegration.simulate(100, flattenedComponent, 2, 1);

        assertTrue(cost >= cost2);
    }

    @Test
    public void monteCarloSimulationPacmanTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH_PACMAN);
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("de.rwth.pacman.heithoff2.controller", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);

        // simulation with spectral Clustering
        double costS = MonteCarloIntegration.simulate(10, flattenedComponent, 10, 1);
        System.out.println("Cost Spectral Clustering: "+costS);

        // simulation with random Clustering
        double costR = MonteCarloIntegration.simulate(10, flattenedComponent, 10, 2);
        System.out.println("Cost Random Clustering: "+costR);

        assertTrue(costR >= costS);
    }


}