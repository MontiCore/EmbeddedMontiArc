package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithmFactory;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringKind;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.*;
import de.monticore.lang.monticar.generator.middleware.helpers.ComponentHelper;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.CommonSymbol;
import net.sf.javaml.clustering.mcl.MarkovClustering;
import net.sf.javaml.clustering.mcl.SparseMatrix;
import net.sf.javaml.core.Dataset;
import net.sf.javaml.core.DefaultDataset;
import net.sf.javaml.core.DenseInstance;
import net.sf.javaml.core.Instance;
import org.junit.Test;
import smile.clustering.DBSCAN;
import smile.clustering.KMeans;
import smile.clustering.SpectralClustering;
import smile.math.distance.MinkowskiDistance;

import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class AutomaticClusteringTest extends AbstractSymtabTest{

    public static final String TEST_PATH = "src/test/resources/";


    @Test
    public void testAdjacencyMatrixCreation(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("lab.system", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        List<ExpandedComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(componentInstanceSymbol);
        double[][] matrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(componentInstanceSymbol),
                ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName));


        //sorted by full name: alex, combine, dinhAn, michael, philipp
        double[][] expRes = {{0,10,0,0,0}
                            ,{10,0,10,10,10}
                            ,{0,10,0,0,0}
                            ,{0,10,0,0,0}
                            ,{0,10,0,0,0}};

        for(int i = 0; i< expRes.length; i++){
            for(int j = 0; j < expRes[i].length;j++){
                assertTrue(expRes[i][j] == matrix[i][j]);
            }
        }
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

    @Test
    public void testFlattenAlgorithm1(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("lab.overallSystem", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        ExpandedComponentInstanceSymbol newComponentInstanceSymbol = AutomaticClusteringHelper.flattenArchitecture(componentInstanceSymbol);
        assertNotNull(newComponentInstanceSymbol);
        Collection<ExpandedComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<ConnectorSymbol> connectors = newComponentInstanceSymbol.getConnectors();
        assertEquals(10, subComponents.size());
        assertEquals(20, connectors.size());
    }

    @Test
    public void testFlattenAlgorithm2(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("lab.spanningSystem", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        ExpandedComponentInstanceSymbol newComponentInstanceSymbol = AutomaticClusteringHelper.flattenArchitecture(componentInstanceSymbol);
        assertNotNull(newComponentInstanceSymbol);
        Collection<ExpandedComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<ConnectorSymbol> connectors = newComponentInstanceSymbol.getConnectors();
        assertEquals(20, subComponents.size());
        assertEquals(40, connectors.size());
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
            }
            testCreateClusters(ClusteringAlgorithmFactory.getFromKind(kind), params);
        }
    }

    private void testCreateClusters(ClusteringAlgorithm algorithm, Object[] params){
        //UnambiguousCluster
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("clustering.unambiguousCluster", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        System.out.println(algorithm);

        List<Set<ExpandedComponentInstanceSymbol>> clusters = null;
        if (params != null) clusters = algorithm.cluster(componentInstanceSymbol, params); else
            clusters = algorithm.cluster(componentInstanceSymbol);


        if (algorithm instanceof SpectralClusteringAlgorithm) {

            assertTrue(clusters.size() == 2);

            Set<ExpandedComponentInstanceSymbol> cluster1 = clusters.get(0);
            Set<ExpandedComponentInstanceSymbol> cluster2 = clusters.get(1);
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

            Set<ExpandedComponentInstanceSymbol> cluster1 = clusters.get(0);
            Set<ExpandedComponentInstanceSymbol> cluster2 = clusters.get(1);
            Set<ExpandedComponentInstanceSymbol> cluster3 = clusters.get(2);
            Set<ExpandedComponentInstanceSymbol> cluster4 = clusters.get(3);
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



    @Test
    public void testClusterToRosConnections() throws IOException {
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        //Force cluster; spectral would not cluster this way!
        List<Set<ExpandedComponentInstanceSymbol>> clusters = new ArrayList<>();

        HashSet<ExpandedComponentInstanceSymbol> cluster1 = new HashSet<>();
        cluster1.add(componentInstanceSymbol.getSubComponent("outComp1").get());
        cluster1.add(componentInstanceSymbol.getSubComponent("inOutComp").get());
        clusters.add(cluster1);

        HashSet<ExpandedComponentInstanceSymbol> cluster2 = new HashSet<>();
        cluster2.add(componentInstanceSymbol.getSubComponent("outComp2").get());
        cluster2.add(componentInstanceSymbol.getSubComponent("doubleInComp").get());
        clusters.add(cluster2);


        AutomaticClusteringHelper.annotateComponentWithRosTagsForClusters(componentInstanceSymbol, clusters);

        List<String> rosPortsSuper = componentInstanceSymbol.getPortsList().stream()
                .filter(PortSymbol::isRosPort)
                .map(PortSymbol::getFullName)
                .collect(Collectors.toList());

        List<String> rosPortsSubComps = componentInstanceSymbol.getSubComponents().stream()
                .flatMap(subc -> subc.getPortsList().stream())
                .filter(PortSymbol::isRosPort)
                .map(PortSymbol::getFullName)
                .collect(Collectors.toList());

        //No Ports in super comp
        assertTrue(rosPortsSuper.size() == 0);
        assertTrue(rosPortsSubComps.size() == 2);

        assertTrue(rosPortsSubComps.contains("clustering.clustersWithSingleConnection.inOutComp.out1"));
        assertTrue(rosPortsSubComps.contains("clustering.clustersWithSingleConnection.doubleInComp.in2"));

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-clustering/ClusterToRosConnections/src/");

        distributedTargetGenerator.add(new CPPGenImpl(),"cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(),"roscpp");

        distributedTargetGenerator.generate(componentInstanceSymbol,taggingResolver);
    }


    @Test
    public void testCostHeuristic(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //CostHeuristic
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("test.costHeuristic", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        double inC = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inC").get());
        double inQ = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inQ").get());
        double inZ = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inZ").get());
        double inB = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inB").get());

        assertTrue(inC > inQ);
        assertTrue(inQ > inZ);
        assertTrue(inZ > inB);

        double inQVec = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inQVec").get());
        double inQVec2 = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inQVec2").get());

        double inQMat = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inQMat").get());
        double inQMat2 = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inQMat2").get());

        assertTrue(inQVec2 > inQVec);
        assertTrue(inQMat2 > inQMat);

        double inPos = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol.getPort("inPos").get());
    }


}
