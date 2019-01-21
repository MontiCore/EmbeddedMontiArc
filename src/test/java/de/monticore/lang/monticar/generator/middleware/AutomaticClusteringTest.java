package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.*;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.*;
import com.clust4j.algo.AffinityPropagation;
import com.clust4j.algo.AffinityPropagationParameters;
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
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.graphstream.graph.implementations.SingleGraph;
import org.graphstream.stream.file.FileSinkImages;
import org.graphstream.ui.view.Viewer;
import org.graphstream.ui.view.ViewerPipe;
import org.junit.Test;
import smile.clustering.DBSCAN;
import smile.clustering.SpectralClustering;
import de.monticore.lang.monticar.svggenerator.SVGMain;
import org.graphstream.graph.*;

import javax.swing.*;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class AutomaticClusteringTest extends AbstractSymtabTest{

    public static final String TEST_PATH = "src/test/resources/";
    public static final String TEST_PATH_PNG = "src/test/resources/clustering/test-images/";


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

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.overallSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol);
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(10, subComponents.size());
        assertEquals(20, connectors.size());
    }

    @Test
    public void testFlattenAlgorithm1ShortNames(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.overallSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol, new HashMap<>());
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(10, subComponents.size());
        assertEquals(20, connectors.size());
    }

    @Test
    public void testFlattenAlgorithm2(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.spanningSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol);
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(20, subComponents.size());
        assertEquals(40, connectors.size());
    }

    @Test
    public void testFlattenAlgorithm2ShortNames() {
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.spanningSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol, new HashMap<>());
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(20, subComponents.size());
        assertEquals(40, connectors.size());
    }

    @Test
    public void testFlattenAlgorithmWithLevels() {
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.spanningSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol, new HashMap<>(), 2);
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(4, subComponents.size());
        assertEquals(24, connectors.size());
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
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //String modelName= "clustering.unambiguousCluster";
        String modelName= "clustering.midSizeDemoCluster";

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve(modelName, ExpandedComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);


        // get stuff together for adjmatrix
        List<ExpandedComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(componentInstanceSymbol);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        Map<Integer, String> subcompsLabels = ComponentHelper.getSubcompsLabels(subcompsOrderedByName);
        double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(componentInstanceSymbol),
                labelsForSubcomps);
        // build a graph from this stuff
        Graph graph = new SingleGraph(modelName);
        Node node= null;
        Edge edge= null;
        String subCompLabel= null;
        for(int i = 0; i < adjMatrix[0].length; i++) {
            node= graph.addNode(Integer.toString(i));
            subCompLabel= subcompsLabels.get(Integer.parseInt(node.getId()));
            subCompLabel= subCompLabel.substring(subCompLabel.lastIndexOf('.') + 1);
            node.addAttribute("ui.label", node.getId() + " (" + subCompLabel + ")");
        }
        for(int i = 0; i < adjMatrix[0].length; i++) {
            for(int j = i; j < adjMatrix[0].length; j++) {
                if (adjMatrix[i][j] > 0) {
                    edge= graph.addEdge(i + "-" + j, Integer.toString(i), Integer.toString(j));
                    edge.addAttribute("ui.label", adjMatrix[i][j]);
                }
            }
        }

        FileSinkImages img = new FileSinkImages(FileSinkImages.OutputType.PNG, FileSinkImages.Resolutions.XGA);
        img.setStyleSheet("graph { padding: 100px; }");
        img.setLayoutPolicy(FileSinkImages.LayoutPolicy.COMPUTED_FULLY_AT_NEW_IMAGE);
        try { img.writeAll(graph, TEST_PATH_PNG + modelName + ".png"); } catch (IOException e) { System.out.println("Couldn't create image file "+TEST_PATH_PNG + modelName + ".png"+
                "\n"+e.getMessage()); };

        SimpleModelViewer viewer= new SimpleModelViewer(graph);
        viewer.run();

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
            testCreateClusters(ClusteringAlgorithmFactory.getFromKind(kind), params, componentInstanceSymbol, modelName);
        }
    }

    private void testCreateClusters(ClusteringAlgorithm algorithm, Object[] params, ExpandedComponentInstanceSymbol componentInstanceSymbol, String modelName){

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.unambiguousCluster", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        System.out.println(algorithm);

        List<Set<EMAComponentInstanceSymbol>> clusters = null;
        if (params != null) clusters = algorithm.cluster(componentInstanceSymbol, params); else
            clusters = algorithm.cluster(componentInstanceSymbol);


        double colorIncrement= 1.0/clusters.size();
        double sizeIncrement= Math.ceil(50/clusters.size());
        double color= 0;
        double size= 10;

        // get stuff together for adjmatrix
        List<ExpandedComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(componentInstanceSymbol);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        Map<Integer, String> subcompsLabels = ComponentHelper.getSubcompsLabels(subcompsOrderedByName);
        double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(componentInstanceSymbol),
                labelsForSubcomps);
        // build a graph from this stuff
        Graph graph = new SingleGraph(algoNameShort);
        Node node= null;
        Edge edge= null;
        String subCompLabel= null;
        for(int i = 0; i < adjMatrix[0].length; i++) {
            node= graph.addNode(Integer.toString(i));
            subCompLabel= subcompsLabels.get(Integer.parseInt(node.getId()));
            subCompLabel= subCompLabel.substring(subCompLabel.lastIndexOf('.') + 1);
            node.addAttribute("ui.label", node.getId() + " (" + subCompLabel + ")");
        }
        for(int i = 0; i < adjMatrix[0].length; i++) {
            for(int j = i; j < adjMatrix[0].length; j++) {
                if (adjMatrix[i][j] > 0) {
                    edge= graph.addEdge(i + "-" + j, Integer.toString(i), Integer.toString(j));
                    edge.addAttribute("ui.label", adjMatrix[i][j]);
                }
            }
        }

        // style (colorize + resize) nodes for clusters
        Node n;
        Edge e;
        String nodeName;
        String nodeId;
        Set<ExpandedComponentInstanceSymbol> cluster;
        List<String> clusterNames;
        for(int i = 0; i < clusters.size(); i++) {
            cluster = clusters.get(i);
            clusterNames = cluster.stream().map(CommonSymbol::getFullName).collect(Collectors.toList());
            for(int j = 0; j < clusterNames.size(); j++) {
                nodeId= null;
                nodeId= labelsForSubcomps.get(clusterNames.get(j)).toString();
                if (nodeId!=null) {
                    n= graph.getNode(nodeId);
                    n.setAttribute("ui.style", "fill-mode: dyn-plain; fill-color: red, black; size: " + size + "px;");
                    n.setAttribute("ui.color", color);

                    // find cutting edges and delete or re-color them
                    for(int k = 0; k < adjMatrix[Integer.parseInt(nodeId)].length; k++) {
                        if (adjMatrix[Integer.parseInt(nodeId)][k] > 0) {
                            // target node k is not in current cluster
                            nodeName= subcompsLabels.get(k);
                            if (!clusterNames.contains(nodeName)) {
                                e= null;
                                e= graph.getEdge(Integer.parseInt(nodeId)+"-"+k);
                                //graph.removeEdge(e);
                                if (e!=null) e.setAttribute("ui.style", "fill-mode: plain; fill-color: #F0F0F0;");
                            }
                        }
                    }
                }
            }
        color= color + colorIncrement;
        size= size + sizeIncrement;
        }

        FileSinkImages img = new FileSinkImages(FileSinkImages.OutputType.PNG, FileSinkImages.Resolutions.XGA);
        img.setStyleSheet("graph { padding: 100px; }");
        img.setLayoutPolicy(FileSinkImages.LayoutPolicy.COMPUTED_FULLY_AT_NEW_IMAGE);
        try { img.writeAll(graph, TEST_PATH_PNG + modelName + "/" + graph.getId() + ".png"); } catch (IOException ex) { System.out.println("Couldn't create image file "+TEST_PATH_PNG + graph.getId() + ".png"+
                "\n"+ex.getMessage()); };

        SimpleModelViewer viewer= new SimpleModelViewer(graph);
        viewer.run();

    if (modelName=="clustering.midSizeDemoCluster") {
        assertTrue(clusters.size() == 2);

        Set<ExpandedComponentInstanceSymbol> cluster1 = clusters.get(0);
        Set<ExpandedComponentInstanceSymbol> cluster2 = clusters.get(1);
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
    public void testClusterToRosConnections() throws IOException {
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        //ClustersWithSingleConnection
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("clustering.clustersWithSingleConnection", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        //Force cluster; spectral would not cluster this way!
        List<Set<EMAComponentInstanceSymbol>> clusters = new ArrayList<>();

        HashSet<EMAComponentInstanceSymbol> cluster1 = new HashSet<>();
        cluster1.add(componentInstanceSymbol.getSubComponent("outComp1").get());
        cluster1.add(componentInstanceSymbol.getSubComponent("inOutComp").get());
        clusters.add(cluster1);

        HashSet<EMAComponentInstanceSymbol> cluster2 = new HashSet<>();
        cluster2.add(componentInstanceSymbol.getSubComponent("outComp2").get());
        cluster2.add(componentInstanceSymbol.getSubComponent("doubleInComp").get());
        clusters.add(cluster2);


        AutomaticClusteringHelper.annotateComponentWithRosTagsForClusters(componentInstanceSymbol, clusters);

        List<String> rosPortsSuper = componentInstanceSymbol.getPortInstanceList().stream()
                .filter(EMAPortInstanceSymbol::isRosPort)
                .map(EMAPortInstanceSymbol::getFullName)
                .collect(Collectors.toList());

        List<String> rosPortsSubComps = componentInstanceSymbol.getSubComponents().stream()
                .flatMap(subc -> subc.getPortInstanceList().stream())
                .filter(EMAPortInstanceSymbol::isRosPort)
                .map(EMAPortInstanceSymbol::getFullName)
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

        AffinityPropagation clustering = new AffinityPropagationParameters().fitNewModel(mat);
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

        AffinityPropagation clustering = new AffinityPropagationParameters().fitNewModel(mat);
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

        AffinityPropagation clustering = new AffinityPropagationParameters().fitNewModel(mat);
        final int[] labels = clustering.getLabels();

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(2, labels.length);
        assertTrue(labels[0] != labels[1]);
    }
}
