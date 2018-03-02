package de.monticore.reporting.svgTools;

import de.monticore.reporting.order.EmamToEma;
import de.monticore.reporting.testCocos.helper.CheckCoCoResult;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class VisualisationHelper {

    private final static File ICON_FOLDER = new File("report/images/icons");

    public static void generateSVGs(List<CheckCoCoResult> testResults, String outputPath, boolean merge) {
        VisualisationHelperMulitThread vHelper = new VisualisationHelperMulitThread();
        vHelper.setThreadNumber(Math.max(Runtime.getRuntime().availableProcessors(), 1));
        vHelper.setTimeout(60);
        File out = new File(outputPath);
        try {
            if (out.exists() && !merge)
                FileUtils.deleteDirectory(out);
            out.mkdirs();
        } catch (IOException e) {
            e.printStackTrace();
        }
        EmamToEma.convertToEma(testResults);
        vHelper.generateSVGs(testResults, outputPath);
    }
}
