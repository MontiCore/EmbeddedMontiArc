package de.monticore.reporting.svgTools;
import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteWatchdog;
import org.apache.commons.exec.PumpStreamHandler;

import java.io.*;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class VisualisationHelperSingleThread implements Runnable {
    private int timeout = 20;
    public TimeUnit timeUnit = TimeUnit.SECONDS;
    private VisualisationHelperMulitThread.Data testResults;
    private String outputPath = "";

    private static final int SUCCESS     = 0;
    private static final int TIMEOUT     = 1;
    private static final int FAILURE     = 2;
    private static final int FAILURE_    = 3;
    private static final int NOTRESOLVED = 4;

    public VisualisationHelperSingleThread(VisualisationHelperMulitThread.Data testResults,
                                           String outputPath){
        this.testResults = testResults;
        this.outputPath  = outputPath;
    }

    @Override
    public void run() {
        SVGInfo tr = testResults.getNextTestResult();
        int z;

        while(tr != null) {
            String fileOut = outputPath + tr.getProject();
            int res = generateSVG(tr, fileOut);
            String msg = "";
            switch(res) {
                case SUCCESS:
                    msg = "Success";
                    break;
                case TIMEOUT:
                    msg = "Timeout";
                    break;
                case FAILURE:
                    msg = "Failure";
                    break;
                case FAILURE_:
                    msg = "Failure";
                    break;
                case NOTRESOLVED:
                    msg = "Could Not Resolve";
                    break;
                default:
                    break;
            }
            z = testResults.getCurrentIndex();
            System.out.println("[" + getFormattedNumber(z, testResults.size()) + "/" + testResults.size() + "]"
                    + " Generating SVG for file " + tr.getModelName() + "\n" + msg);

            tr = testResults.getNextTestResult();
        }
    }

    private int generateSVG(SVGInfo testResult, String outputPath) {
        if (testResult.getResolved() != 1) return NOTRESOLVED;
        testResult.addErrorMessage("[INFO] do SVG generation<br>=========================");
        File out = new File(outputPath);
        if (!out.exists())
            out.mkdirs();
        try {
            String modelPath = testResult.isEmaModelPathAvailable() ? testResult.getEmaModelPath() :
                    testResult.getModelPath();
            generateSVG(modelPath, testResult.getQualifiedName(), outputPath);
            testResult.setSvgPath(outputPath + testResult.getQualifiedName());
            testResult.addErrorMessage("[INFO] SVG generation success<br>");
            return SUCCESS;
        } catch (TimeoutException e) {
            testResult.addErrorMessage("[ERROR] SVG generation timed out<br>");
            testResult.addErrorMessage(e.getMessage());
            testResults.incrementTimeOuts();
            return TIMEOUT;
        } catch (SVGGenerationException e) {
            testResult.addErrorMessage("[ERROR] SVG generation failed<br>");
            testResult.addErrorMessage(e.getMessage());
            return FAILURE;
        } catch (Exception e) {
            testResult.addErrorMessage("[ERROR] SVG generation failed<br>");
            testResult.addErrorMessage(e.getMessage());
            return FAILURE_;
        }
    }

    private void generateSVG(String modelPath, String modelName, String outputPath) throws TimeoutException, IOException, SVGGenerationException {
        boolean timeouted = false;
        boolean failed = false;
        boolean success = true;

        String[] args = {
                "java", "-jar",
                "svgGenerator/montiarc-svggenerator-4.0.1-SNAPSHOT-jar-with-dependencies.jar",
                "--input", modelName,
                "--modelPath", modelPath,
                "--recursiveDrawing", "true",
                "--outputPath", outputPath
        };
        String command = "";
        for(String str: args) {
            command += str + " ";
        }

        long timeStart = System.currentTimeMillis();

        ByteArrayOutputStream stdoutOS = new ByteArrayOutputStream();
        ByteArrayOutputStream stderrOS = new ByteArrayOutputStream();

        DefaultExecutor executor = new DefaultExecutor();
        PumpStreamHandler pumpStreamHandler = new PumpStreamHandler(stdoutOS, stderrOS);
        ExecuteWatchdog watchDog = new ExecuteWatchdog(timeout * 1000);
        executor.setStreamHandler(pumpStreamHandler);
        executor.setWatchdog(watchDog);
        executor.execute(CommandLine.parse(command));

        long timeEnd = System.currentTimeMillis();
        long timeNeeded = timeEnd - timeStart;

        if(timeNeeded/1000 > timeout) {
            timeouted = true;
        }

        String output = deleteWarning(stdoutOS.toString() + "\n" + stderrOS.toString());

        if (!output.contains("Overall: success")) {
            failed = true;
            testResults.setTimeErrored(timeNeeded, modelPath + modelName);
        } else if(output.contains("exception") || output.contains("Overall: failure")) {
            failed = true;
            testResults.setTimeErrored(timeNeeded, modelPath + modelName);
        } else if (!timeouted){
            testResults.setTime(timeNeeded, modelPath + modelName);
        }


        String argsString = Arrays.toString(args).replace("[","").replace("]","").replace(",", "");
        if (timeouted)
            throw new TimeoutException("<br>" + "command used: " + argsString + "<br>" + output);
        if (failed)
            throw new SVGGenerationException("<br>" + "command used: " + argsString + "<br>"  + output);

    }

    private String deleteWarning(String str){
        return str
                .replace("SLF4J: Failed to load class \"org.slf4j.impl.StaticLoggerBinder\".\r\n", "")
                .replace("SLF4J: Defaulting to no-operation (NOP) logger implementation\r\n", "")
                .replace("SLF4J: See http://www.slf4j.org/codes.html#StaticLoggerBinder for further details.\r\n", "")
                .replace("SLF4J: Failed to load class \"org.slf4j.impl.StaticLoggerBinder\".\r", "")
                .replace("SLF4J: Defaulting to no-operation (NOP) logger implementation\r", "")
                .replace("SLF4J: See http://www.slf4j.org/codes.html#StaticLoggerBinder for further details.\r", "")
                .replace("SLF4J: Failed to load class \"org.slf4j.impl.StaticLoggerBinder\".\n", "")
                .replace("SLF4J: Defaulting to no-operation (NOP) logger implementation\n", "")
                .replace("SLF4J: See http://www.slf4j.org/codes.html#StaticLoggerBinder for further details.\n", "")
                .replace("Picked up _JAVA_OPTIONS: -Xmx2048m -Xms512m","")
                .replaceAll("copying icon jar.*icon_full_detail.svg(\r\n|\r|\n)*","")
                .replaceAll("copying icon jar.*icon_no_port_names.svg(\r\n|\r|\n)*","")
                .replaceAll("copying icon jar.*icon_simplified.svg(\r\n|\r|\n)*","")
        ;
    }

    private String getFormattedNumber(int z, int max) {
        int blanks = ((int) Math.log10(max)) - ((int) Math.log10(z));
        String blankString = "";
        for (int i = 0; i < blanks; i++)
            blankString += " ";
        return blankString + z;
    }

    public void setTimeout(int timeout) {
        this.timeout = timeout;
    }
}
