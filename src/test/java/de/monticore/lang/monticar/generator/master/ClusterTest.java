package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertNotNull;

public class ClusterTest extends AbstractSymtabTest {

    @Test
    public void testSingleComponentClusters() {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.distComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        List<Set<ExpandedComponentInstanceSymbol>> clusters = ClusterHelper.getClusters(componentInstanceSymbol);
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

        List<Set<ExpandedComponentInstanceSymbol>> clusters = ClusterHelper.getClusters(componentInstanceSymbol);

        Set<ExpandedComponentInstanceSymbol> cluster1 = new HashSet<>();
        Set<ExpandedComponentInstanceSymbol> cluster2 = new HashSet<>();

        cluster1.add(componentInstanceSymbol.getSubComponent("sub1").orElse(null));
        cluster2.add(componentInstanceSymbol.getSubComponent("sub2").orElse(null));
        cluster2.add(componentInstanceSymbol.getSubComponent("sub3").orElse(null));

        assertTrue(clusters.contains(cluster1));
        assertTrue(clusters.contains(cluster2));
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

        ClusterHelper.getClusters(componentInstanceSymbol);

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

        List<Set<ExpandedComponentInstanceSymbol>> clusters = ClusterHelper.getClusters(componentInstanceSymbol);
        assertTrue(clusters.size() == 1);

        Set<ExpandedComponentInstanceSymbol> cluster1 = new HashSet<>();
        cluster1.add(componentInstanceSymbol.getSubComponent("sub1").orElse(null));

        assertTrue(clusters.contains(cluster1));
    }
}
