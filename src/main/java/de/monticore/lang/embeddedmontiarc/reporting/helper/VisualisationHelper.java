package de.monticore.lang.embeddedmontiarc.reporting.helper;

import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class VisualisationHelper {

    private static int timeouts = 0;
    private static int timeout = 60;
    private static TimeUnit timeUnit = TimeUnit.SECONDS;
    private static long maxTime = -1;
    private static long maxTimeErrored = 0;
    private static String maxTimeFile = "";

    public static void generateSVGs(List<TestResult> testResults, String outputPath) {
        String output = outputPath.replace("\\", "/");
        String current = System.getProperty("user.dir").replace("\\", "/");
        output = output.substring(current.length() + 1);
        if (output.charAt(output.length() - 1) != '/')
            output = output + "/";
        File out = new File(output);
        if (!out.exists())
            out.mkdirs();
        int z = 1;
        for (TestResult tr : testResults) {
            System.out.println("[" + getFormattedNumber(z, testResults.size()) + "/" + testResults.size() + "]"
                    + " Generating SVG for file " + tr.getModelName());
            String fileOut = output + tr.getProject();
            generateSVG(tr, fileOut);
            z++;
        }
        System.out.println("Timeouts: " + timeouts);
        System.out.println("Max time needed: " + ((double) maxTime) / 1000 + " s");
        System.out.println("Time needed for File: " + maxTimeFile);
    }

    private static String getFormattedNumber(int z, int max) {
        int blanks = ((int) Math.log10(max)) - ((int) Math.log10(z));
        String blankString = "";
        for (int i = 0; i < blanks; i++)
            blankString += " ";
        return blankString + z;
    }

    private static void generateSVG(TestResult testResult, String outputPath) {
        if (testResult.getResolve() != 1) return;
        testResult.addErrorMessage("[INFO] do SVG generation<br>=========================");
        File out = new File(outputPath);
        if (!out.exists())
            out.mkdirs();
        try {
            generateSVG(testResult.getModelPath(), testResult.getQualifiedName(), outputPath);
            testResult.setSvgPath(outputPath + testResult.getQualifiedName());
            testResult.addErrorMessage("[INFO] SVG generation success<br>");
        } catch (TimeoutException e) {
            testResult.addErrorMessage("[ERROR] SVG generation timed out<br>");
            testResult.addErrorMessage(e.getMessage());
            timeouts++;
        } catch (SVGGenerationException e) {
            testResult.addErrorMessage("[ERROR] SVG generation failed<br>");
            testResult.addErrorMessage(e.getMessage());
        } catch (Exception e) {
            testResult.addErrorMessage("[ERROR] SVG generation failed<br>");
            testResult.addErrorMessage(e.getMessage());
        }
    }

    private static void generateSVG(String modelPath, String modelName, String outputPath) throws InterruptedException, TimeoutException, IOException, SVGGenerationException {
        String[] args = {
                "java", "-jar",
                "montiarc-svggenerator-4.0.1-SNAPSHOT-jar-with-dependencies.jar",
                "--input", modelName,
                "--onlineIDE",
                "--modelPath", modelPath,
                "--recursiveDrawing", "true",
                "--outputPath", outputPath
        };
        long timeStart = System.currentTimeMillis();
        Process ps = Runtime.getRuntime().exec(args);
        if (!ps.waitFor(timeout, timeUnit)) {
            java.io.InputStream is = ps.getInputStream();
            byte b[] = new byte[is.available()];
            is.read(b, 0, b.length);
            //timeout - kill the process.
            ps.destroy(); // consider using destroyForcibly instead
            System.out.println("Timeout");
            throw new TimeoutException("<br>" + new String(b));
        }

        long timeEnd = System.currentTimeMillis();
        long timeNeeded = timeEnd - timeStart;

        File out = new File(outputPath + "/" + modelName + ".html");
        if (!out.exists()) {
            java.io.InputStream is = ps.getInputStream();
            byte b[] = new byte[is.available()];
            is.read(b, 0, b.length);

            if (timeNeeded > maxTimeErrored)
                maxTimeErrored = timeNeeded;

            throw new SVGGenerationException("<br>" + new String(b));
        }

        if (timeNeeded > maxTime) {
            maxTime = timeNeeded;
            maxTimeFile = modelPath + modelName;
        }

    }

    public static void setTimeout(int timeout) {
        VisualisationHelper.timeout = timeout;
    }

    public static void setTimeUnit(TimeUnit timeUnit) {
        VisualisationHelper.timeUnit = timeUnit;
    }

    public static void init(String modelPath, String modelName) {
        try {
            generateSVG(modelPath, modelName, "testOut");
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (TimeoutException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (SVGGenerationException e) {
            e.printStackTrace();
        }
        long time = maxTime > 0 ? maxTime : maxTimeErrored;
        timeout = ((int) time/1000) + 1;
        maxTimeErrored = - 1;
        maxTime = -1;
    }
}
