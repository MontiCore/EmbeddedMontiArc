package de.monticore.lang.embeddedmontiarc.reporting.helper;

import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;

import java.io.File;
import java.util.List;

public class VisualisationHelper {

    public static void generateSVGs(List<TestResult> testResults, String outputPath) {
        String output = outputPath.replace("\\", "/");
        String current = System.getProperty("user.dir").replace("\\","/");
        output = output.substring(current.length()+1);
        if(output.charAt(output.length() - 1) != '/')
            output = output + "/";
        File out = new File(output);
        if (!out.exists())
            out.mkdirs();
        for (TestResult tr: testResults) {
            System.out.println("Generating SVG for file " + tr.getModelName());
            String fileOut = output + tr.getProject();
            generateSVG(tr, fileOut);
        }
    }

    public static void generateSVG(TestResult testResult, String outputPath) {
        if(testResult.getResolve() != 1) return;
        File out = new File(outputPath);
        if (!out.exists())
            out.mkdirs();
        try {
            generateSVG(testResult.getModelPath(), testResult.getQualifiedName(), outputPath);
            testResult.setSvgPath(outputPath + testResult.getQualifiedName());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    protected static void generateSVG (String modelPath, String modelName, String outputPath){
        String[] args = {
                "java","-jar",
                "montiarc-svggenerator-4.0.1-SNAPSHOT-jar-with-dependencies.jar",
                "--input", modelName,
                "--onlineIDE",
                "--modelPath", modelPath,
                "--recursiveDrawing", "true",
                "--outputPath", outputPath
        };
        try {
            Process ps=Runtime.getRuntime().exec(args);
            ps.waitFor();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
