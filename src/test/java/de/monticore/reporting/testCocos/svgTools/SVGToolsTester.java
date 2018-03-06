package de.monticore.reporting.testCocos.svgTools;

import de.monticore.reporting.order.OrderTestResults;
import de.monticore.reporting.svgTools.VisualisationHelper;
import de.monticore.reporting.testCocos.CheckCoCos;
import de.monticore.reporting.testCocos.helper.CheckCoCoResult;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SVGToolsTester {

    @Test
    public void testSVGgeneration() throws IOException {
        String output = "target/generatedTestResults/SVG/";
        String root = "src/test/";
        String zipDummy = "modelsDummy.zip";
        String compareSVG = "src/test/resources/comparisonResults/SVG/";
        File rootFile = new File(root);
        CheckCoCos tcc = new CheckCoCos();
        List<CheckCoCoResult> testResults = tcc.testAllCocos(rootFile, zipDummy, "ema");
        OrderTestResults order = new OrderTestResults();
        order.orderTestResults(testResults);
        List<CheckCoCoResult> rootModels = order.getRootModels();

        VisualisationHelper.generateSVGs(testResults, rootModels, output, false);

        File out = new File(output + "/resources");
        File toCompare = new File(compareSVG);

        assertTrue("Output not found.", out.exists() && out.isDirectory());
        assertTrue("Compare SVG not found.", toCompare.exists() && toCompare.isDirectory());
        assertEquals("Wrong result size.", toCompare.listFiles().length, out.listFiles().length);

        for(File fG: out.listFiles()){
            boolean found = false;
            if( fG.isDirectory() ) continue;
            for(File fC: toCompare.listFiles()){
                if(fG.getName().equals(fC.getName())){
                    found = true;
                }
            }
            assertTrue("SVG file \" " + fG.getName() + " \" not found.", found);
        }
    }
}
