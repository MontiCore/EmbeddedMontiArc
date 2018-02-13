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
    private static String maxTimeErroredFile = "";
    private static int falsePositive = 0;

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
        System.out.println("Timeouts: " + timeouts + " \twith timeout limit set to: " + timeout);
        System.out.println("Max time needed: " + ((double) maxTime) / 1000 + " s");
        System.out.println("Time needed for file: " + maxTimeFile);
        System.out.println("Max time needed for errored file: " + ((double) maxTimeErrored) / 1000 + " s");
        System.out.println("Time needed for file: " + maxTimeErroredFile);
//        System.out.println("False negative: " + falsePositive);
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
        boolean timeouted = false;
        boolean failed = false;
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
            //timeout - kill the process.
            ps.destroy(); // consider using destroyForcibly instead
            System.out.println("Timeout");
            timeouted = true;
        }

        long timeEnd = System.currentTimeMillis();
        long timeNeeded = timeEnd - timeStart;

        java.io.InputStream is = ps.getInputStream();
        java.io.InputStream err = ps.getErrorStream();
        byte b1[] = new byte[is.available()];
        is.read(b1, 0, b1.length);
        byte b2[] = new byte[err.available()];
        err.read(b2, 0, b2.length);
        String inputString = new String(b1);
        String errorString = new String(b2);
        errorString = errorString
                .replace("SLF4J: Failed to load class \"org.slf4j.impl.StaticLoggerBinder\".\r\n", "")
                .replace("SLF4J: Defaulting to no-operation (NOP) logger implementation\r\n", "")
                .replace("SLF4J: See http://www.slf4j.org/codes.html#StaticLoggerBinder for further details.\r\n", "")
                .replace("SLF4J: Failed to load class \"org.slf4j.impl.StaticLoggerBinder\".\r", "")
                .replace("SLF4J: Defaulting to no-operation (NOP) logger implementation\r", "")
                .replace("SLF4J: See http://www.slf4j.org/codes.html#StaticLoggerBinder for further details.\r", "")
                .replace("SLF4J: Failed to load class \"org.slf4j.impl.StaticLoggerBinder\".\n", "")
                .replace("SLF4J: Defaulting to no-operation (NOP) logger implementation\n", "")
                .replace("SLF4J: See http://www.slf4j.org/codes.html#StaticLoggerBinder for further details.\n", "")
        ;

        String checkString = errorString
                .replace("\t", "").replace(" ", "")
                .replace("\r\n", "").replace("\r","").replace("\n","");
//        File out = new File(outputPath + "/" + modelName + ".html");
//
//        if (!timeouted && (!out.exists() || !checkString.equals(""))) {
//            failed = true;
//            if (timeNeeded > maxTimeErrored) {
//                maxTimeErrored = timeNeeded;
//                maxTimeErroredFile = modelPath + modelName;
//            }
//            if (checkString.equals("")) {
//                falsePositive ++;
//            }
//        } else if (!timeouted){
//            if (timeNeeded > maxTime) {
//                maxTime = timeNeeded;
//                maxTimeFile = modelPath + modelName;
//            }
//        }

        if (!checkString.equals("")) {
            failed = true;
            if (timeNeeded > maxTimeErrored) {
                maxTimeErrored = timeNeeded;
                maxTimeErroredFile = modelPath + modelName;
            }
        } else if (!timeouted){
            if (timeNeeded > maxTime) {
                maxTime = timeNeeded;
                maxTimeFile = modelPath + modelName;
            }
        }

        if (timeouted)
            throw new TimeoutException("<br>" + inputString + "<br>" + errorString);
        if (failed)
            throw new SVGGenerationException("<br>" + inputString + "<br>" + errorString);

    }

    public static void setTimeout(int timeout) {
        VisualisationHelper.timeout = timeout;
    }

    public static void setTimeUnit(TimeUnit timeUnit) {
        VisualisationHelper.timeUnit = timeUnit;
    }

    public static void init(String modelPath, String modelName) {
        try {
            generateSVG(modelPath, modelName, "testOut/");
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (TimeoutException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (SVGGenerationException e) {
        }
        long time = maxTime > 0 ? maxTime : maxTimeErrored;
        timeout = ((int) time / 1000) + 1;
        maxTimeErrored = -1;
        maxTime = -1;
        System.out.println("Timeout set to: " + timeout + "s");
    }
}
