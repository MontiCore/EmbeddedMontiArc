/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering;

import com.clust4j.algo.AffinityPropagation;
import com.clust4j.algo.AffinityPropagationParameters;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.algorithms.*;
import de.monticore.lang.monticar.clustering.qualityMetric.SilhouetteIndex;
import de.monticore.lang.monticar.clustering.visualization.ModelVisualizer;
import de.monticore.lang.monticar.clustering.helpers.ComponentHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.CommonSymbol;
import net.sf.javaml.clustering.mcl.MarkovClustering;
import net.sf.javaml.clustering.mcl.SparseMatrix;
import net.sf.javaml.core.Dataset;
import net.sf.javaml.core.DefaultDataset;
import net.sf.javaml.core.DenseInstance;
import net.sf.javaml.core.Instance;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.graphstream.graph.Graph;
import org.junit.Ignore;
import org.junit.Test;
import smile.clustering.DBSCAN;
import smile.clustering.SpectralClustering;

import java.util.*;
import java.util.stream.Collectors;

import static org.junit.Assert.*;

public class AutomaticClusteringTest extends AbstractSymtabTest{

    public static final String TEST_PATH = "src/test/resources/";
    public static final String TEST_PATH_PNG = "target/clustering/test-images/";

    @Test
    public void testAdjacencyMatrixCreation(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("lab.system", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        List<EMAComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(componentInstanceSymbol);
        double[][] matrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(componentInstanceSymbol),
                ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName));


        //sorted by full name: alex, combine, dinhAn, michael, philipp
        double[][] expRes = {{0,8,0,0,0}
                            ,{8,0,8,8,8}
                            ,{0,8,0,0,0}
                            ,{0,8,0,0,0}
                            ,{0,8,0,0,0}};

        for(int i = 0; i< expRes.length; i++){
            for(int j = 0; j < expRes[i].length;j++){
                assertTrue(expRes[i][j] == matrix[i][j]);
            }
        }
    }

    @Test
    public void testAdjacencyMatrixCreation2(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("lab.adjMatrixComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        List<EMAComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(componentInstanceSymbol);
        double[][] matrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(componentInstanceSymbol),
                ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName));


        //sorted full name: sub1, sub2, sub3
        double[][] expRes = {{0,8,16}  //sub1
                            ,{8,0,0}  //sub2
                            ,{16,0,0}}; //sub3

        for(int i = 0; i< expRes.length; i++){
            for(int j = 0; j < expRes[i].length;j++){
                assertTrue(expRes[i][j] == matrix[i][j]);
            }
        }
    }


    @Test
    public void testDistanceMatrixCreation(){
        // a -(10)-> b -(20)-> c    |    d

        double[][] adj = {
                            {0, 10, 0, 0},
                            {10, 0, 20, 0},
                            {0, 20, 0, 0},
                            {0, 0, 0, 0}};

        double[][] dist = AutomaticClusteringHelper.getDistanceMatrix(adj);

        double m = Double.MAX_VALUE;
        double[][] expDist = {
                {0, 10, 30, m},
                {10, 0, 20, m},
                {30, 20, 0, m},
                {m, m, m, 0}};


        for(int i = 0; i< expDist.length; i++){
            for(int j = 0; j < expDist[i].length;j++){
                assertTrue(expDist[i][j] == dist[i][j]);
            }
        }

    }


    @Test
    public void testSilhouetteIndex(){
        // graph:
        // a,b close to each other
        // c,d close to each other
        // big difference between a and c as well as b and d

        double[][] adjMat = {
                {0, 10, 1000, 1000},
                {10, 0, 1000, 1000},
                {1000, 1000, 0, 10},
                {1000, 1000, 10, 0}
        };

        double[][] dist = AutomaticClusteringHelper.getDistanceMatrix(adjMat);

        int[] correctCustering = {0,0,1,1};
        SilhouetteIndex index1 = new SilhouetteIndex(dist, correctCustering);
        assertTrue(index1.S(0) > 0.5);
        assertTrue(index1.S(1) > 0.5);
        assertTrue(index1.S(2) > 0.5);
        assertTrue(index1.S(3) > 0.5);


        int[] badClustering = {0,0,0,1};
        SilhouetteIndex index2 = new SilhouetteIndex(dist, badClustering);
        assertTrue(index2.S(0) > 0.5);
        assertTrue(index2.S(1) > 0.5);
        assertTrue(index2.S(2) < -0.5);
        assertTrue(index2.S(3) > 0.5);

        assertTrue(index1.getSilhouetteScore() > index2.getSilhouetteScore());
    }


    @Test
    public void testSpectralClustering(){
        
        // 0 1 0 0
        // 1 0 0 0
        // 0 0 0 1
        // 0 0 1 0
        
        //zu 2 cluster -> (a,b) (c,d)
        
        double[][] adjMatrix =  {{0, 1, 0, 0},
                                 {1, 0, 0, 0},
                                 {0, 0, 0, 1},
                                 {0, 0, 1, 0}};

        SpectralClustering clustering = new SpectralClustering(adjMatrix,2);

        int[] labels = clustering.getClusterLabel();

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(4, labels.length);
        assertTrue(labels[0] == labels[1]);
        assertTrue(labels[2] == labels[3]);
        assertTrue( labels[0] != labels[2]);
        assertTrue( labels[0] != labels[3]);
        assertTrue( labels[1] != labels[2]);
        assertTrue( labels[1] != labels[3]);

    }

    @Ignore
    @Test
    public void testSpectralClusteringIsolatedVertex(){
        double[][] adjMatrix =  {
                                {0, 1, 1, 0},
                                {1, 0, 0, 0},
                                {1, 0, 0, 0},
                                {0, 0, 0, 0}};

        SpectralClustering clustering = new SpectralClustering(adjMatrix,2);
    }


    // todo: gotta move this thing later, just temporarily here for testing purposes
    public static Dataset[] getClustering(Dataset data, SparseMatrix matrix) {
        int[] sparseMatrixSize = matrix.getSize();
        int attractors = 0;

        // just for testing/debugging purposes
        for(int i = 0; i < sparseMatrixSize[0]; ++i) {
            double val = matrix.get(i, i);
            if (val != 0.0D) {
                ++attractors;
            }
        }

        Vector<Vector<Instance>> finalClusters = new Vector();

        for(int i = 0; i < sparseMatrixSize[0]; ++i) {
            Vector<Instance> cluster = new Vector();
            double val = matrix.get(i, i);
            if (val >= 0.98D) {
                for(int j = 0; j < sparseMatrixSize[0]; ++j) {
                    double value = matrix.get(j, i);
                    if (value != 0.0D) {
                        cluster.add(data.instance(j));
                    }
                }

                finalClusters.add(cluster);
            }
        }

        Dataset[] output = new Dataset[finalClusters.size()];

        int i;
        for(i = 0; i < finalClusters.size(); ++i) {
            output[i] = new DefaultDataset();
        }

        for(i = 0; i < finalClusters.size(); ++i) {
            new Vector();
            Vector<Instance> getCluster = (Vector)finalClusters.get(i);

            for(int j = 0; j < getCluster.size(); ++j) {
                output[i].add((Instance)getCluster.get(j));
            }
        }

        return output;
    }

    @Test
    public void testDBSCANClustering(){

        /*

        0----1----4---6
        | \/ |     \ /
        | /\ |      5
        2----3

        expected: 2 clusters a, b with a={0,1,2,3} and b={4,5,6}

        */

        // for DBSCAN this could be directly weighted
        double[][] adjacencyMatrix =
                {
                        {0, 1, 1, 1, 0, 0, 0},
                        {1, 0, 1, 1, 1, 0, 0},
                        {1, 1, 0, 1, 0, 0, 0},
                        {1, 1, 1, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 1, 1},
                        {0, 0, 0, 0, 1, 0, 1},
                        {0, 0, 0, 0, 1, 1, 0},
                };


        // |nodes| instances of data with pseudo x,y coords. set to node no.
        double[][] data = new double[adjacencyMatrix.length][2];
        for (int i=0; i<data.length; i++) {
            data[i][0]= i;
            data[i][1]= i;
        }

        // mission critical
        int minPts= 2;
        double radius= 5;

        DBSCAN clustering = new DBSCAN(data, new DBSCANDistance(adjacencyMatrix), minPts, radius);

        int[] labels = clustering.getClusterLabel();

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(7, labels.length);
        assertTrue(labels[0] == labels[1]);
        assertTrue(labels[0] == labels[2]);
        assertTrue(labels[0] == labels[3]);

        assertTrue(labels[1] == labels[0]);
        assertTrue(labels[1] == labels[2]);
        assertTrue(labels[1] == labels[3]);
        assertTrue(labels[1] != labels[4]);     // expected cut

        assertTrue(labels[2] == labels[0]);
        assertTrue(labels[2] == labels[1]);
        assertTrue(labels[2] == labels[3]);

        assertTrue(labels[3] == labels[0]);
        assertTrue(labels[3] == labels[1]);
        assertTrue(labels[3] == labels[2]);

        assertTrue(labels[4] != labels[1]);     // expected cut
        assertTrue(labels[4] == labels[5]);
        assertTrue(labels[4] == labels[6]);

        assertTrue(labels[5] == labels[4]);
        assertTrue(labels[5] == labels[6]);

        assertTrue(labels[6] == labels[4]);
        assertTrue(labels[6] == labels[5]);

    }

    @Test
    public void testWeightedAdjacencyMatrix2transitionMatrix() {

        /*

        0----1----4---6
        | \/ |     \ /
        | /\ |      5
        2----3

        weights used for testing: 1, 5, 10, 20, 50

        */

        double[][] adjacencyMatrix =
                {
                        {0, 1, 5, 20, 0, 0, 0},
                        {1, 0, 1, 1, 50, 0, 0},
                        {5, 1, 0, 1, 0, 0, 0},
                        {20, 1, 1, 0, 0, 0, 0},
                        {0, 50, 0, 0, 0, 1, 1},
                        {0, 0, 0, 0, 1, 0, 1},
                        {0, 0, 0, 0, 1, 1, 0},
                };

        double[][] transitionMatrix = AutomaticClusteringHelper.weightedAdjacencyMatrix2transitionMatrix(adjacencyMatrix);

        double minValWeight;
        double minValProb;
        double ratioWeight;
        double ratioProb;
        double sum;
        for(int i = 0; i < transitionMatrix[0].length; i++) {
            System.out.println();
            minValWeight= minValProb= Double.MAX_VALUE;
            sum= 0;
            // calculate sum and smallest value >0 in transitionMatrix
            for (int j = 0; j < transitionMatrix[0].length; j++) {
                System.out.print(transitionMatrix[i][j] + " ");
                sum+= transitionMatrix[i][j];
                if (adjacencyMatrix[i][j]>0) {
                    if (adjacencyMatrix[i][j] < minValWeight) minValWeight= adjacencyMatrix[i][j];
                }
            }
            // expectation: each row sums up to exactly 1
            System.out.print(" SUM: " + sum);
            assertEquals(1.0, sum, 0);
            // find smallest value >0 in adjacencyMatrix
            for (int j = 0; j < transitionMatrix[0].length; j++) {
                if (adjacencyMatrix[i][j]>0) {
                    if (adjacencyMatrix[i][j] < minValProb) minValProb= adjacencyMatrix[i][j];
                }
            }
            // check pairwise ratio of probabilities
            for (int j = 0; j < transitionMatrix[0].length; j++) {
                if (transitionMatrix[i][j]>0) {
                    ratioWeight= minValWeight/adjacencyMatrix[i][j];
                    ratioProb= minValProb/transitionMatrix[i][j];
                    // expectation: the ratio between the smallest weight and each row weight in the adjacencyMatrix is the same
                    // as the ratio between the smallest probability and each row probability in the transitionMatrix
                    assertEquals(ratioWeight, ratioProb, 0);
                }
            }
        }

    }

    @Test
    public void testMarkovClustering(){

        /*

        0----1----4---6
        | \/ |     \ /
        | /\ |      5
        2----3

        expected: 2 clusters a, b with a={0,1,2,3} and b={4,5,6}

        */

//        row-major order (for java-ml)
//        0 1 1 1 0 0 0
//        1 0 1 1 1 0 0
//        1 1 0 1 0 0 0
//        1 1 1 0 0 0 0
//        0 1 0 0 0 1 1
//        0 0 0 0 1 0 1
//        0 0 0 0 1 1 0

        double[][] adjacencyMatrix =
                {
                        {0, 1, 1, 1, 0, 0, 0},
                        {1, 0, 1, 1, 1, 0, 0},
                        {1, 1, 0, 1, 0, 0, 0},
                        {1, 1, 1, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 1, 1},
                        {0, 0, 0, 0, 1, 0, 1},
                        {0, 0, 0, 0, 1, 1, 0},
                };

        // expected trans. matrix
        /*
        double[][] transitionMatrix =
            {
                {  0, .33, .33, .33,   0,   0,   0},
                {.25,   0, .25, .25, .25,   0,   0},
                {.33, .33,   0, .33,   0,   0,   0},
                {.33, .33, .33,   0,   0,   0,   0},
                {  0, .33,   0,   0,   0, .33, .33},
                {  0,   0,   0,   0,  .5,   0,  .5},
                {  0,   0,   0,   0,  .5,  .5,   0}
            };
        */
        double[][] transitionMatrix = AutomaticClusteringHelper.adjacencyMatrix2transitionMatrix(adjacencyMatrix);

        // |nodes| instances of data with one attribute denoting the node order
        Dataset original_ds= new DefaultDataset();
        for (int i=0; i<adjacencyMatrix[0].length; i++) {
            original_ds.add(new DenseInstance(new double[]{i}));
        }

        SparseMatrix smatrix = new SparseMatrix(transitionMatrix);
        MarkovClustering mcl = new MarkovClustering();
        double maxResidual = 0.001;
        double gammaExp = 2.0;
        double loopGain = 0.;
        double zeroMax = 0.001;
        SparseMatrix matrix = mcl.run(smatrix, maxResidual, gammaExp, loopGain, zeroMax);

        Dataset[] clustered_ds= getClustering(original_ds, matrix);

        // translate for monti stuff
        int data_point;
        int[] labels = new int[original_ds.size()];
        for (int cluster=0; cluster < clustered_ds.length; cluster++) {
            for (int instance=0; instance < clustered_ds[cluster].size(); instance++) {
                data_point= clustered_ds[cluster].instance(instance).get(0).intValue();
                labels[data_point]= cluster;
            }
        }

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(7, labels.length);
        assertTrue(labels[0] == labels[1]);
        assertTrue(labels[0] == labels[2]);
        assertTrue(labels[0] == labels[3]);

        assertTrue(labels[1] == labels[0]);
        assertTrue(labels[1] == labels[2]);
        assertTrue(labels[1] == labels[3]);
        assertTrue(labels[1] != labels[4]);     // expected cut

        assertTrue(labels[2] == labels[0]);
        assertTrue(labels[2] == labels[1]);
        assertTrue(labels[2] == labels[3]);

        assertTrue(labels[3] == labels[0]);
        assertTrue(labels[3] == labels[1]);
        assertTrue(labels[3] == labels[2]);

        assertTrue(labels[4] != labels[1]);     // expected cut
        assertTrue(labels[4] == labels[5]);
        assertTrue(labels[4] == labels[6]);

        assertTrue(labels[5] == labels[4]);
        assertTrue(labels[5] == labels[6]);

        assertTrue(labels[6] == labels[4]);
        assertTrue(labels[6] == labels[5]);

    }

    @Test
    public void testClusteringAlgorithms(){

        //String modelName= "clustering.unambiguousCluster";
        String modelName= "clustering.midSizeDemoCluster";

        EMAComponentInstanceSymbol componentInstanceSymbol = ModelVisualizer.loadModel(TEST_PATH, modelName);

        assertNotNull(componentInstanceSymbol);


        Graph graph = ModelVisualizer.buildGraph(componentInstanceSymbol, modelName);

        ModelVisualizer.saveGraphAsImage(graph, TEST_PATH_PNG, modelName);
        // ModelVisualizer.viewGraph(graph);

        Object[] params;
        for(ClusteringKind kind : ClusteringKind.values()){
            params= null;
            switch (kind) {
                case SPECTRAL_CLUSTERER:
                    params= new Object[] { SpectralClusteringBuilder.SpectralParameters.SPECTRAL_NUM_CLUSTERS, 2 };
                break;
                case DBSCAN_CLUSTERER:
                    params= new Object[] {
                            DBSCANClusteringBuilder.DBSCANParameters.DBSCAN_MIN_PTS, 1,
                            DBSCANClusteringBuilder.DBSCANParameters.DBSCAN_RADIUS, 10.0
                    };
                    break;
                case AFFINITY_CLUSTERER:
                    params= new Object[]{
                            AffinityPropagationBuilder.AffinityPropagationParameters.SEED, 104L
                    };
            }
            testCreateClusters(ClusteringAlgorithmFactory.getFromKind(kind), params, componentInstanceSymbol, modelName);
        }
    }

    private void testCreateClusters(ClusteringAlgorithm algorithm, Object[] params, EMAComponentInstanceSymbol componentInstanceSymbol, String modelName){

        String algoName= algorithm.toString().substring(0, algorithm.toString().lastIndexOf("@"));
        String algoNameShort= algoName.substring(algoName.lastIndexOf(".")+1);
        System.out.println(algoName);

        assertNotNull(componentInstanceSymbol);

        List<Set<EMAComponentInstanceSymbol>> clusters = null;
        ClusteringInput clusteringInput = new ClusteringInput(componentInstanceSymbol);
        if (params != null){
            clusters = algorithm.cluster(clusteringInput, params);
        }else{
            clusters = algorithm.cluster(clusteringInput);
        }

        Graph graph = ModelVisualizer.buildGraph(componentInstanceSymbol, algoNameShort);
        ModelVisualizer.visualizeClustering(graph, clusters, componentInstanceSymbol);

        ModelVisualizer.saveGraphAsImage(graph, TEST_PATH_PNG, modelName + "/" + graph.getId());
        // ModelVisualizer.viewGraph(graph);

        if (modelName=="clustering.midSizeDemoCluster") {
            if (algorithm instanceof AffinityPropagationAlgorithm) {
                assertTrue(clusters.get(0).size() > 0 &&
                        clusters.get(1).size() > 0 &&
                        clusters.get(2).size() > 0
                );

                Set<EMAComponentInstanceSymbol> cluster1 = clusters.get(0);
                Set<EMAComponentInstanceSymbol> cluster2 = clusters.get(1);
                Set<EMAComponentInstanceSymbol> cluster3 = clusters.get(2);
                assertTrue((cluster1.size() == 3 && cluster2.size() == 1 && cluster3.size() == 3) ||
                        (cluster1.size() == 3 && cluster2.size() == 3 && cluster3.size() == 1) ||
                        (cluster1.size() == 1 && cluster2.size() == 3 && cluster3.size() == 3)
                );

                List<String> cluster1Names = cluster1.stream()
                        .map(CommonSymbol::getFullName)
                        .collect(Collectors.toList());

                List<String> cluster2Names = cluster2.stream()
                        .map(CommonSymbol::getFullName)
                        .collect(Collectors.toList());

                List<String> cluster3Names = cluster3.stream()
                        .map(CommonSymbol::getFullName)
                        .collect(Collectors.toList());

                // cut-off point should be at comp1 or comp4, so those nodes should form a cluster of their own
                if (cluster1.size() == 1) {
                    if (cluster1Names.get(0).endsWith("comp1")) assertTrue(cluster1Names.contains(modelName + ".comp1"));
                    if (cluster1Names.get(0).endsWith("comp4")) assertTrue(cluster1Names.contains(modelName + ".comp4"));
                } else if (cluster2.size() == 1) {
                    if (cluster2Names.get(0).endsWith("comp1")) assertTrue(cluster2Names.contains(modelName + ".comp1"));
                    if (cluster2Names.get(0).endsWith("comp4")) assertTrue(cluster2Names.contains(modelName + ".comp4"));
                } else if (cluster3.size() == 1) {
                    if (cluster3Names.get(0).endsWith("comp1")) assertTrue(cluster3Names.contains(modelName + ".comp1"));
                    if (cluster3Names.get(0).endsWith("comp4")) assertTrue(cluster3Names.contains(modelName + ".comp4"));
                }

            } else {
                assertTrue(clusters.size() == 2);

                Set<EMAComponentInstanceSymbol> cluster1 = clusters.get(0);
                Set<EMAComponentInstanceSymbol> cluster2 = clusters.get(1);
                assertTrue((cluster1.size() == 3 && cluster2.size() == 4) ||
                        (cluster2.size() == 3 && cluster1.size() == 4)
                );

                List<String> cluster1Names = cluster1.stream()
                        .map(CommonSymbol::getFullName)
                        .collect(Collectors.toList());

                List<String> cluster2Names = cluster2.stream()
                        .map(CommonSymbol::getFullName)
                        .collect(Collectors.toList());

                if (cluster1.size() == 4) {
                    if (cluster1Names.get(0).endsWith("comp0") ||
                            cluster1Names.get(0).endsWith("comp1") ||
                            cluster1Names.get(0).endsWith("comp2") ||
                            cluster1Names.get(0).endsWith("comp3")
                    ) {
                        assertTrue(cluster1Names.contains(modelName + ".comp0"));
                        assertTrue(cluster1Names.contains(modelName + ".comp1"));
                        assertTrue(cluster1Names.contains(modelName + ".comp2"));
                        assertTrue(cluster1Names.contains(modelName + ".comp3"));

                        assertTrue(cluster2Names.contains(modelName + ".comp4"));
                        assertTrue(cluster2Names.contains(modelName + ".comp5"));
                        assertTrue(cluster2Names.contains(modelName + ".comp6"));
                    }
                } else if (cluster1.size() == 3) {
                    if (cluster1Names.get(0).endsWith("comp4") ||
                            cluster1Names.get(0).endsWith("comp5") ||
                            cluster1Names.get(0).endsWith("comp6")
                    ) {
                        assertTrue(cluster2Names.contains(modelName + ".comp0"));
                        assertTrue(cluster2Names.contains(modelName + ".comp1"));
                        assertTrue(cluster2Names.contains(modelName + ".comp2"));
                        assertTrue(cluster2Names.contains(modelName + ".comp3"));

                        assertTrue(cluster1Names.contains(modelName + ".comp4"));
                        assertTrue(cluster1Names.contains(modelName + ".comp5"));
                        assertTrue(cluster1Names.contains(modelName + ".comp6"));
                    }
                }
            }
        }
        if (modelName=="clustering.unambiguousCluster") {
            if (algorithm instanceof SpectralClusteringAlgorithm) {

                assertTrue(clusters.size() == 2);

                Set<EMAComponentInstanceSymbol> cluster1 = clusters.get(0);
                Set<EMAComponentInstanceSymbol> cluster2 = clusters.get(1);
                assertTrue(cluster1.size() == 2);
                assertTrue(cluster2.size() == 2);

                List<String> cluster1Names = cluster1.stream()
                        .map(CommonSymbol::getFullName)
                        .collect(Collectors.toList());

                List<String> cluster2Names = cluster2.stream()
                        .map(CommonSymbol::getFullName)
                        .collect(Collectors.toList());

                if (cluster1Names.get(0).endsWith("compA") || cluster1Names.get(0).endsWith("compB")) {
                    assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compA"));
                    assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compB"));

                    assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compC"));
                    assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compD"));
                } else {
                    assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compC"));
                    assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compD"));

                    assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compA"));
                    assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compB"));
                }
            }

            if (algorithm instanceof MarkovClusteringAlgorithm) {

                assertTrue(clusters.size() == 4);

                Set<EMAComponentInstanceSymbol> cluster1 = clusters.get(0);
                Set<EMAComponentInstanceSymbol> cluster2 = clusters.get(1);
                Set<EMAComponentInstanceSymbol> cluster3 = clusters.get(2);
                Set<EMAComponentInstanceSymbol> cluster4 = clusters.get(3);
                assertTrue(cluster1.size() == 1);
                assertTrue(cluster2.size() == 1);
                assertTrue(cluster3.size() == 1);
                assertTrue(cluster4.size() == 1);

                List<String> cluster1Names = cluster1.stream().map(CommonSymbol::getFullName).collect(Collectors.toList());
                List<String> cluster2Names = cluster2.stream().map(CommonSymbol::getFullName).collect(Collectors.toList());
                List<String> cluster3Names = cluster3.stream().map(CommonSymbol::getFullName).collect(Collectors.toList());
                List<String> cluster4Names = cluster4.stream().map(CommonSymbol::getFullName).collect(Collectors.toList());

                assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compA"));
                assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compB"));
                assertTrue(cluster3Names.contains("clustering.unambiguousCluster.compC"));
                assertTrue(cluster4Names.contains("clustering.unambiguousCluster.compD"));
            }
        }

    }

    @Test
    public void testCostHeuristic(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //CostHeuristic
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("test.costHeuristic", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        double inC = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inC").get());
        double inQ = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inQ").get());
        double inZ = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inZ").get());
        double inB = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inB").get());

        assertTrue(inC > inQ);
        assertTrue(inQ > inZ);
        assertTrue(inZ > inB);

        double inQVec = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inQVec").get());
        double inQVec2 = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inQVec2").get());

        double inQMat = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inQMat").get());
        double inQMat2 = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inQMat2").get());

        assertTrue(inQVec2 > inQVec);
        assertTrue(inQMat2 > inQMat);

        double inPos = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPortInstance("inPos").get());
    }

    @Test
    public void testAffinityPropagation(){
        // Nodes: 4
        double[][] adjMatrix =  {{0, 0, 1, 0},
                {0, 0, 0, 1},
                {1, 0, 0, 0},
                {0, 1, 0, 0}};

        RealMatrix mat = new Array2DRowRealMatrix(adjMatrix);

        AffinityPropagation clustering = new AffinityPropagationParameters().setSeed(new Random(104)).fitNewModel(mat);
        final int[] labels = clustering.getLabels();

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(4, labels.length);
        assertTrue(labels[0] != labels[1]);
        assertTrue(labels[0] == labels[2]);
        assertTrue(labels[0] != labels[3]);
        assertTrue(labels[1] != labels[0]);
        assertTrue(labels[1] != labels[2]);
        assertTrue(labels[1] == labels[3]);
        assertTrue(labels[2] == labels[0]);
        assertTrue(labels[2] != labels[1]);
        assertTrue(labels[2] != labels[3]);
    }

    @Test
    public void testAffinityPropagation2(){
        // Nodes: 4
        double[][] adjMatrix =  {{0, 0, 1, 0},
                {0, 0, 0, 5},
                {1, 0, 0, 0},
                {0, 5, 0, 0}};

        RealMatrix mat = new Array2DRowRealMatrix(adjMatrix);

        AffinityPropagation clustering = new AffinityPropagationParameters().setSeed(new Random(104)).fitNewModel(mat);
        final int[] labels = clustering.getLabels();

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(4, labels.length);
        assertTrue(labels[0] != labels[1]);
        assertTrue(labels[0] == labels[2]);
        assertTrue(labels[0] == labels[3]);
        assertTrue(labels[1] != labels[0]);
        assertTrue(labels[1] != labels[2]);
        assertTrue(labels[1] != labels[3]);
        assertTrue(labels[2] == labels[0]);
        assertTrue(labels[2] != labels[1]);
        assertTrue(labels[2] == labels[3]);
    }

    @Test
    public void testAffinityPropagation3(){
        // Nodes: 2
        // 0 1
        // 1 0

        double[][] adjMatrix =  {{1, 0},
                {0, 1}};

        RealMatrix mat = new Array2DRowRealMatrix(adjMatrix);

        AffinityPropagation clustering = new AffinityPropagationParameters().setSeed(new Random(104)).fitNewModel(mat);
        final int[] labels = clustering.getLabels();

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(2, labels.length);
        assertTrue(labels[0] != labels[1]);
    }


}
