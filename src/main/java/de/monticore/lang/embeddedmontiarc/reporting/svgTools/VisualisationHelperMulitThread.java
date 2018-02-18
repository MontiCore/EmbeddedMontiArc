package de.monticore.lang.embeddedmontiarc.reporting.svgTools;

import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;

import java.io.File;
import java.util.Iterator;
import java.util.List;

public class VisualisationHelperMulitThread {

    private int timeout = 60;
    private int threadNumber = 1;

    public void generateSVGs(List<TestResult> testResults, String outputPath) {
        this.threadNumber = 4 * Runtime.getRuntime().availableProcessors();
        String output = outputPath.replace("\\", "/");
        String current = System.getProperty("user.dir").replace("\\", "/");
        output = output.substring(current.length() + 1);
        if (output.charAt(output.length() - 1) != '/')
            output = output + "/";
        File out = new File(output);
        if (!out.exists())
            out.mkdirs();

        Data data = startThreads(testResults, output);
        System.out.println("\n<==============SVG-generation==============>");
        System.out.println("Timeouts: " + data.timeouts + " \twith timeout limit set to: " + timeout + "s");
        System.out.println("Max time needed: " + ((double) data.maxTime) / 1000 + " s");
        System.out.println("Time needed for file: " + data.maxTimeFile);
        System.out.println("Max time needed for errored file: " + ((double) data.maxTimeErrored) / 1000 + " s");
        System.out.println("Time needed for file: " + data.maxTimeErroredFile);
    }

    private Data startThreads(List<TestResult> testResults, String outputPath) {
        Data data = new Data(testResults);
        Thread[] threads = new Thread[this.threadNumber];
        for (int i = 0; i < threads.length; i++) {
            VisualisationHelperSingleThread vh = new VisualisationHelperSingleThread(data, outputPath);
            vh.setTimeout(timeout);
            threads[i] = new Thread(vh);
        }

        long timeStart = System.currentTimeMillis();
        for (Thread thread : threads) {
            thread.start();
        }

        int running;
        do {
            running = 0;
            for (Thread thread : threads) {
                if (thread.isAlive()) {
                    running++;
                }
            }
        } while (running > 0);

        long timeEnd = System.currentTimeMillis();
        long timeNeeded = timeEnd - timeStart;
        System.out.println("\nTime with " + threadNumber + " threads: " + timeNeeded / 1000 + "s");
        return data;
    }

    public class Data {
        private List<TestResult> testResults;
        private Iterator<TestResult> iterator;
        private int z = 0;

        public int timeouts = 0;
        public long maxTime = -1;
        public long maxTimeErrored = -1;
        public String maxTimeFile = "";
        public String maxTimeErroredFile = "";

        public Data(List<TestResult> testResults) {
            this.testResults = testResults;
            this.iterator = testResults.iterator();
        }

        public synchronized TestResult getNextTestResult() {
            if (!iterator.hasNext()) return null;
            return iterator.next();
        }

        public synchronized int getCurrentIndex() {
            return ++z;
        }

        public synchronized void setTime(long timeNeeded, String file) {
            if (timeNeeded > maxTime) {
                maxTime = timeNeeded;
                maxTimeFile = file;
            }
        }

        public synchronized void setTimeErrored(long timeNeeded, String file) {
            if (timeNeeded > maxTimeErrored) {
                maxTimeErrored = timeNeeded;
                maxTimeErroredFile = file;
            }
        }

        public synchronized void incrementTimeOuts() {
            timeouts++;
        }

        public int size() {
            return testResults.size();
        }
    }

    public void setTimeout(int timeout) {
        this.timeout = timeout;
    }
}
