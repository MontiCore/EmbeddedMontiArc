package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.monticar.generator.middleware.cli.CliParametersLoader;
import de.monticore.lang.monticar.generator.middleware.cli.CliParameters;
import de.monticore.lang.monticar.generator.middleware.cli.ClustringParameters;
import de.monticore.lang.monticar.generator.middleware.cli.ResultChoosingStrategy;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.*;
import org.junit.Test;

import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;


public class ParameterLoadingTest {
    private static String BASE_DIR = "src/test/resources/config/parameterTest/";

    private CliParameters loadCliParameters(String fileName) throws FileNotFoundException {
        return CliParametersLoader.loadCliParameters(BASE_DIR + fileName + ".json");
    }

    @Test
    public void testClusterParamsDefaultValues() throws FileNotFoundException {
        CliParameters params = loadCliParameters("clusterParamsDefaultValues");
        ClustringParameters clustringParameters = params.getClustringParameters().get();
        assertFalse(clustringParameters.getNumberOfClusters().isPresent());
        assertEquals(ResultChoosingStrategy.bestWithFittingN, clustringParameters.getChooseBy());
        assertTrue(clustringParameters.getAlgorithmParameters().isEmpty());
    }

    @Test
    public void testClusterParamsNoAlgo() throws FileNotFoundException {
        CliParameters params = loadCliParameters("clusterParamsNoAlgo");
        ClustringParameters clustringParameters = params.getClustringParameters().get();
        assertEquals(3, (long) clustringParameters.getNumberOfClusters().get());
        assertEquals(ResultChoosingStrategy.bestOverall, clustringParameters.getChooseBy());
        assertTrue(clustringParameters.getAlgorithmParameters().isEmpty());
    }

    @Test
    public void testClusterParamsAllAlgos() throws FileNotFoundException {
        CliParameters params = loadCliParameters("clusterParamsAllAlgos");
        ClustringParameters clustringParameters = params.getClustringParameters().get();

        List<AlgorithmCliParameters> algorithmParameters = clustringParameters.getAlgorithmParameters();
        assertEquals(4, algorithmParameters.size());

        Map<String, AlgorithmCliParameters> nameToParams = new HashMap<>();
        algorithmParameters.forEach(p -> nameToParams.put(p.getName(), p));

        assertTrue(nameToParams.get("SpectralClustering") instanceof SpectralClusteringCliParameters);
        assertTrue(nameToParams.get("DBScan") instanceof DBScanCliParameters);
        assertTrue(nameToParams.get("Markov") instanceof MarkovCliParameters);
        assertTrue(nameToParams.get("AffinityPropagation") instanceof AffinityPropagationCliParameters);
    }

     @Test
    public void testClusterParamsInvalidAlgos() throws FileNotFoundException {
        CliParameters params = loadCliParameters("clusterParamsInvalidAlgos");
        ClustringParameters clustringParameters = params.getClustringParameters().get();

        List<AlgorithmCliParameters> algorithmParameters = clustringParameters.getAlgorithmParameters();
        assertEquals(2, algorithmParameters.size());

        assertTrue(algorithmParameters.get(0) instanceof UnknownAlgorithmCliParameters);
        assertTrue(algorithmParameters.get(1) instanceof UnknownAlgorithmCliParameters);
    }





}
