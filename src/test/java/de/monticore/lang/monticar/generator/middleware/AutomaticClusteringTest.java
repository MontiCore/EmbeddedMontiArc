package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.helpers.ComponentHelper;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.CommonSymbol;
import org.junit.Test;
import smile.clustering.SpectralClustering;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
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
        double[][] expRes = {{0,1,0,0,0}
                            ,{1,0,1,1,1}
                            ,{0,1,0,0,0}
                            ,{0,1,0,0,0}
                            ,{0,1,0,0,0}};

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
    public void testCreateClusters(){
        //UnambiguousCluster
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("clustering.unambiguousCluster", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        SpectralClusteringAlgorithm spectralClusteringAlgorithm = new SpectralClusteringAlgorithm();
        List<Set<ExpandedComponentInstanceSymbol>> clusters = spectralClusteringAlgorithm.cluster(componentInstanceSymbol, 2);

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

        if(cluster1Names.get(0).endsWith("compA") || cluster1Names.get(0).endsWith("compB")){
            assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compA"));
            assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compB"));

            assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compC"));
            assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compD"));
        }else{
            assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compC"));
            assertTrue(cluster1Names.contains("clustering.unambiguousCluster.compD"));

            assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compA"));
            assertTrue(cluster2Names.contains("clustering.unambiguousCluster.compB"));
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

}
