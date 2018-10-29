package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.emadl.generator.EMADLAbstractSymtab;
import de.monticore.lang.monticar.generator.middleware.helpers.ClusterHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

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

        double[][] matrix = ClusterHelper.createAdjacencyMatrix(componentInstanceSymbol);


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




}
