package de.monticore.reporting.svgTools;

import de.monticore.reporting.testCocos.helper.CheckCoCoResult;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class VisualisationHelper {

    private final static File ICON_FOLDER = new File("report/images/icons");

    public static void generateSVGs(List<CheckCoCoResult> testResults, String outputPath) {
        VisualisationHelperMulitThread vHelper = new VisualisationHelperMulitThread();
        vHelper.setThreadNumber(Math.max(Runtime.getRuntime().availableProcessors(), 1));
        vHelper.setTimeout(60);
        File out = new File(outputPath);
        try {
            if (out.exists())
                FileUtils.deleteDirectory(out);
            out.mkdirs();
        } catch (IOException e) {
            e.printStackTrace();
        }
        vHelper.generateSVGs(testResults, outputPath);
        if (ICON_FOLDER.exists())
            copyIcons(new File(outputPath));

//        setSVGPathForRootFiles(testResults);
    }

    private static void setSVGPathForRootFiles(List<CheckCoCoResult> testResults) {
        for(CheckCoCoResult rootFile: testResults){
            if(rootFile.getResolved() != 1) continue;
            if(rootFile.getSvgPath() == "") continue;
            String svgHome = rootFile.getSvgPath();
            svgHome = svgHome.replace("\\","/").substring(0, svgHome.lastIndexOf("/") + 1);
            setSVGPath(rootFile, svgHome);
        }
    }

    private static void setSVGPath(CheckCoCoResult testResult, String svgHome) {
        if(testResult.getSvgPath().equals("") && testResult.getChildren().size() > 0){
            testResult.setSvgPath(svgHome + testResult.getQualifiedName() + ".html");
        }
        for(CheckCoCoResult child: testResult.getChildren()){
            setSVGPath(child, svgHome);
        }
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
