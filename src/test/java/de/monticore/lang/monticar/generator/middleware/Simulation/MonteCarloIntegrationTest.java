package de.monticore.lang.monticar.generator.middleware.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.AbstractSymtabTest;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.List;
import java.util.Set;

import static org.junit.Assert.*;

public class MonteCarloIntegrationTest {

    public static final String TEST_PATH = "src/test/resources/";
    public static final String TEST_PATH_PACMAN = "src/test/resources/pacman/";

    @Test
    public void costRandomClustering(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        List<Set<EMAComponentInstanceSymbol>> clusters = MonteCarloIntegration.randomClustering(componentInstanceSymbol, 2);

        assertTrue("Too many or less clusters created.", clusters.size() == 2);
        assertTrue("Subcomponent 1 is not distributed evenly/correctly!", clusters.get(0).size()>=1);
        assertTrue("Subcomponent 2 is not distributed evenly/correctly!",clusters.get(1).size()>=1);
    }

    @Test
    public void mcSimulationTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol componentInstanceSymbol2 = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        // random clustering
        double cost = MonteCarloIntegration.simulate(100, componentInstanceSymbol, 2, 2);

        assertTrue(cost >= 10);
    }

    @Test
    public void mcSimulationPacmanTest(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH_PACMAN);
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("de.rwth.pacman.heithoff2.controller", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        // simulation with random Clustering
        double cost = MonteCarloIntegration.simulate(10, componentInstanceSymbol, 10, 2);
    }


}