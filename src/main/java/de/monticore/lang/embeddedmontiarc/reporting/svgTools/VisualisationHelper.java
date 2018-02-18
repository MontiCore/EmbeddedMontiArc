package de.monticore.lang.embeddedmontiarc.reporting.svgTools;

import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class VisualisationHelper {

    private final static File ICON_FOLDER = new File("report/images/icons");

    public static void generateSVGs(List<TestResult> testResults, String outputPath) {
        VisualisationHelperMulitThread vHelper = new VisualisationHelperMulitThread();
        vHelper.generateSVGs(testResults, outputPath);
        if(ICON_FOLDER.exists())
            copyIcons(new File(outputPath));
    }

    private static void copyIcons(File root) {
        if (!root.exists() || !root.isDirectory()) return;
        for (File subDir : root.listFiles()) {
            if (subDir.isDirectory()) {
                copyIcons(subDir);
            }
        }
        if (!root.getName().equals("SVG")) {
            File dest = new File(root.getAbsolutePath() + "/icons");
            try {
                FileUtils.copyDirectory(ICON_FOLDER, dest);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
