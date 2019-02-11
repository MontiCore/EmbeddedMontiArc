package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.monticar.generator.middleware.cli.CliParameters;
import de.monticore.lang.monticar.generator.middleware.cli.CliParametersLoader;
import de.monticore.lang.monticar.generator.middleware.cli.ClusteringParameters;
import de.monticore.lang.monticar.generator.middleware.cli.ResultChoosingStrategy;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.*;
import org.junit.Test;

import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.*;


public class ParameterLoadingTest {
    private static String BASE_DIR = "src/test/resources/config/parameterTest/";

    private CliParameters loadCliParameters(String fileName) throws FileNotFoundException {
        return CliParametersLoader.loadCliParameters(BASE_DIR + fileName + ".json");
    }

    @Test
    public void testClusterParamsDefaultValues() throws FileNotFoundException {
        CliParameters params = loadCliParameters("clusterParamsDefaultValues");
        ClusteringParameters clusteringParameters = params.getClusteringParameters().get();
        assertFalse(clusteringParameters.getNumberOfClusters().isPresent());
        assertEquals(ResultChoosingStrategy.bestWithFittingN, clusteringParameters.getChooseBy());
        assertTrue(clusteringParameters.getAlgorithmParameters().isEmpty());
    }

    @Test
    public void testClusterParamsNoAlgo() throws FileNotFoundException {
        CliParameters params = loadCliParameters("clusterParamsNoAlgo");
        ClusteringParameters clusteringParameters = params.getClusteringParameters().get();
        assertEquals(3, (long) clusteringParameters.getNumberOfClusters().get());
        assertEquals(ResultChoosingStrategy.bestOverall, clusteringParameters.getChooseBy());
        assertTrue(clusteringParameters.getAlgorithmParameters().isEmpty());
    }

    @Test
    public void testClusterParamsAllAlgos() throws FileNotFoundException {
        CliParameters params = loadCliParameters("clusterParamsAllAlgos");
        ClusteringParameters clusteringParameters = params.getClusteringParameters().get();

        List<AlgorithmCliParameters> algorithmParameters = clusteringParameters.getAlgorithmParameters();
        assertEquals(4, algorithmParameters.size());

        Map<String, AlgorithmCliParameters> nameToParams = new HashMap<>();
        algorithmParameters.forEach(p -> nameToParams.put(p.getName(), p));


        assertTrue(nameToParams.get("SpectralClustering") instanceof SpectralClusteringCliParameters);
        assertTrue(nameToParams.get("DBScan") instanceof DBScanCliParameters);
        assertTrue(nameToParams.get("Markov") instanceof MarkovCliParameters);
        assertTrue(nameToParams.get("AffinityPropagation") instanceof AffinityPropagationCliParameters);

        SpectralClusteringCliParameters spectralClustering = (SpectralClusteringCliParameters) nameToParams.get("SpectralClustering");
        DBScanCliParameters dbScan = (DBScanCliParameters) nameToParams.get("DBScan");
        MarkovCliParameters markov = (MarkovCliParameters) nameToParams.get("Markov");
        AffinityPropagationCliParameters affinityPropagation = (AffinityPropagationCliParameters) nameToParams.get("AffinityPropagation");

        assertTrue(spectralClustering.isValid());
        assertTrue(dbScan.isValid());
        assertTrue(markov.isValid());
        assertTrue(affinityPropagation.isValid());

        double delta = 0.000001;

        assertEquals(4, (int) spectralClustering.getNumberOfClusters().get());
        assertEquals(10, (int) spectralClustering.getL().get());
        assertEquals(0.01, (double) spectralClustering.getSigma().get(), delta);

        assertEquals(5, (int) dbScan.getMinPts().get());
        assertEquals(2.5, (double) dbScan.getRadius().get(), delta);

        assertEquals(0.001, markov.getMaxResidual().get(), delta);
        assertEquals(2.0, markov.getGammaExp().get(), delta);
        assertEquals(0, markov.getLoopGain().get(), delta);
        assertEquals(0.002, markov.getZeroMax().get(), delta);

    }

    @Test
    public void testClusterParamsInvalidAlgos() throws FileNotFoundException {
        CliParameters params = loadCliParameters("clusterParamsInvalidAlgos");
        ClusteringParameters clusteringParameters = params.getClusteringParameters().get();

        List<AlgorithmCliParameters> algorithmParameters = clusteringParameters.getAlgorithmParameters();
        assertEquals(2, algorithmParameters.size());

        assertTrue(algorithmParameters.get(0) instanceof UnknownAlgorithmCliParameters);
        assertTrue(algorithmParameters.get(1) instanceof UnknownAlgorithmCliParameters);
    }

    @Test
    public void testClusterParamsNumberOfClustersOverride() throws FileNotFoundException {
        CliParameters params = loadCliParameters("NumberOfClustersOverride");
        ClusteringParameters clusteringParameters = params.getClusteringParameters().get();

        List<AlgorithmCliParameters> algorithmParameters = clusteringParameters.getAlgorithmParameters();
        assertEquals(4, algorithmParameters.size());

        assertTrue(algorithmParameters.get(0) instanceof SpectralClusteringCliParameters);
        assertTrue(algorithmParameters.get(1) instanceof SpectralClusteringCliParameters);
        assertTrue(algorithmParameters.get(2) instanceof SpectralClusteringCliParameters);

        SpectralClusteringCliParameters sc0 = (SpectralClusteringCliParameters) algorithmParameters.get(0);
        SpectralClusteringCliParameters sc1 = (SpectralClusteringCliParameters) algorithmParameters.get(1);
        SpectralClusteringCliParameters sc2 = (SpectralClusteringCliParameters) algorithmParameters.get(2);

        assertEquals(5, (int) sc0.getNumberOfClusters().get());
        assertEquals(5, (int) sc1.getNumberOfClusters().get());
        assertEquals(5, (int) sc2.getNumberOfClusters().get());
    }





}
