package de.monticore.lang.embeddedmontiarc.reporting.svgTools;

import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;

import java.io.File;
import java.io.IOException;
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
        TestResult tr = testResults.getNextTestResult();
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

    private int generateSVG(TestResult testResult, String outputPath) {
        if (testResult.getResolve() != 1) return NOTRESOLVED;
        testResult.addErrorMessage("[INFO] do SVG generation<br>=========================");
        File out = new File(outputPath);
        if (!out.exists())
            out.mkdirs();
        try {
            generateSVG(testResult.getModelPath(), testResult.getQualifiedName(), outputPath);
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

    private void generateSVG(String modelPath, String modelName, String outputPath) throws InterruptedException, TimeoutException, IOException, SVGGenerationException {
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
        ps.destroy();
        String inputString = new String(b1);
        String errorString = new String(b2);
        errorString = deleteWarning(errorString);

        String checkString = errorString
                .replace("\t", "").replace(" ", "")
                .replace("\r\n", "").replace("\r","").replace("\n","");

        if (!checkString.equals("")) {
            failed = true;
            testResults.setTimeErrored(timeNeeded, modelPath + modelName);
        } else if (!timeouted){
            testResults.setTime(timeNeeded, modelPath + modelName);
        }


        if (timeouted)
            throw new TimeoutException("<br>" + inputString + "<br>" + errorString);
        if (failed)
            throw new SVGGenerationException("<br>" + inputString + "<br>" + errorString);

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
