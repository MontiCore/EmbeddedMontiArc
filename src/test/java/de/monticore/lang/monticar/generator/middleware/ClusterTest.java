package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.middleware.helpers.ClusterFromTagsHelper;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;

public class ClusterTest extends AbstractSymtabTest {

    @Test
    public void testSingleComponentClusters() {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.distComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        List<Set<ExpandedComponentInstanceSymbol>> clusters = ClusterFromTagsHelper.getClusters(componentInstanceSymbol);
        assertTrue(clusters.size() == 2);

        Set<ExpandedComponentInstanceSymbol> cluster1 = new HashSet<>();
        Set<ExpandedComponentInstanceSymbol> cluster2 = new HashSet<>();

        cluster1.add(componentInstanceSymbol.getSubComponent("sub1").orElse(null));
        cluster2.add(componentInstanceSymbol.getSubComponent("sub2").orElse(null));

        assertTrue(clusters.contains(cluster1));
        assertTrue(clusters.contains(cluster2));
    }

    @Test
    public void testMultiComponentCluster() {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.twoCompCluster", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        List<Set<ExpandedComponentInstanceSymbol>> clusters = ClusterFromTagsHelper.getClusters(componentInstanceSymbol);

        Set<ExpandedComponentInstanceSymbol> cluster1 = new HashSet<>();
        Set<ExpandedComponentInstanceSymbol> cluster2 = new HashSet<>();

        cluster1.add(componentInstanceSymbol.getSubComponent("sub1").orElse(null));
        cluster2.add(componentInstanceSymbol.getSubComponent("sub2").orElse(null));
        cluster2.add(componentInstanceSymbol.getSubComponent("sub3").orElse(null));

        assertTrue(clusters.contains(cluster1));
        assertTrue(clusters.contains(cluster2));

        List<ExpandedComponentInstanceSymbol> clusterComps = ClusterFromTagsHelper.getClusterSubcomponents(componentInstanceSymbol);
        assertTrue(clusterComps.size() == 2);

        ExpandedComponentInstanceSymbol clusterComp1 = clusterComps.get(0);
        ExpandedComponentInstanceSymbol clusterComp2 = clusterComps.get(1);

        assertTrue(clusterComp1.getName().equals("sub1"));
        assertTrue(clusterComp1.getPort("rosOut").isPresent());

        PortSymbol rosIn = clusterComp2.getPort("rosIn").orElse(null);
        ExpandedComponentInstanceSymbol sub2 = clusterComp2.getSubComponent("sub2").orElse(null);
        ExpandedComponentInstanceSymbol sub3 = clusterComp2.getSubComponent("sub3").orElse(null);

        assertNotNull(rosIn);
        assertNotNull(sub2);
        assertNotNull(sub3);

        PortSymbol sub2NoRosOut = sub2.getPort("noRosOut").orElse(null);
        PortSymbol sub3NoRosIn = sub3.getPort("noRosIn").orElse(null);
        PortSymbol sub3RosIn = sub3.getPort("rosIn").orElse(null);

        assertNotNull(sub2NoRosOut);
        assertNotNull(sub3NoRosIn);
        assertNotNull(sub3RosIn);

        assertFalse(sub2NoRosOut.getMiddlewareSymbol().isPresent());
        assertFalse(sub3NoRosIn.getMiddlewareSymbol().isPresent());
        assertTrue(sub3RosIn.getMiddlewareSymbol().isPresent());
    }

    @Test
    public void generateMultiComponentCluster() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.twoCompCluster", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-cmake/multiCompCluster/src/");
        distributedTargetGenerator.add(new CPPGenImpl(), "cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(), "roscpp");

        List<File> files = distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testInvalidSuperConnection() {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.invalidSuperConnection", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        Log.enableFailQuick(false);
        Log.getFindings().clear();

        ClusterFromTagsHelper.getClusters(componentInstanceSymbol);

        List<Finding> findings = Log.getFindings();

        assertTrue(findings.size() == 1);
        assertTrue(findings.get(0).getMsg().contains("0x8EFC3"));

        findings.clear();
    }

    @Test
    public void testValidSuperConnection() {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.validSuperConnection", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        List<Set<ExpandedComponentInstanceSymbol>> clusters = ClusterFromTagsHelper.getClusters(componentInstanceSymbol);
        assertTrue(clusters.size() == 1);

        Set<ExpandedComponentInstanceSymbol> cluster1 = new HashSet<>();
        cluster1.add(componentInstanceSymbol.getSubComponent("sub1").orElse(null));

        assertTrue(clusters.contains(cluster1));
    }
}
