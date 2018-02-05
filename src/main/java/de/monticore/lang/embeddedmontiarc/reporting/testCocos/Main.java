package de.monticore.lang.embeddedmontiarc.reporting.testCocos;

import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResultPrinter;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestsEndWithTestResult;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.util.List;

public class Main {
    public static void main(String[] args){
        ReportContext context = getContext(args);
        if(context.isTestCoCos()){
            TestCoCos tcc = new TestCoCos();
            List<TestResult> testResults = tcc.testAllCocos(new File(context.getProjectRoot()), "ema","emam");
            TestResultPrinter.printTestResults(testResults, context.getOutput() + "/data.json");
        }
        if(context.isTestsEndWithTest()) {
            TestsEndWithTest tewt = new TestsEndWithTest();
            List<TestsEndWithTestResult> testResults = tewt.testTestsEndWithTest(new File(context.getProjectRoot()));
            TestResultPrinter.printTestsEndWithTestResults(testResults, context.getOutput() + "/dataEWT.json");
        }
    }

    protected static class ReportContext {
        private boolean testsEndWithTest = false;
        private boolean testCoCos = true;
        private String output = "";
        private String projectRoot = "";

        public boolean isTestsEndWithTest() {
            return testsEndWithTest;
        }

        public void setTestsEndWithTest(boolean testsEndWithTest) {
            this.testsEndWithTest = testsEndWithTest;
        }

        public boolean isTestCoCos() {
            return testCoCos;
        }

        public void setTestCoCos(boolean testCoCos) {
            this.testCoCos = testCoCos;
        }

        public String getOutput() {
            return output;
        }

        public void setOutput(String output) {
            this.output = output;
        }

        public String getProjectRoot() {
            return projectRoot;
        }

        public void setProjectRoot(String projectRoot) {
            this.projectRoot = projectRoot;
        }
    }

    private static String help() {
        return
                "\n\nUsage: reporting projectRoot outputPath [options]\n\n" +
                        "PARAMETERS\n" +
                        "  projectRoot       Points to the directory with all projects in\n" +
                        "  outputPath        Points to the directory with the html file in\n" +
                        "OPTIONS\n" +
                        "  -h --help         Prints this help page\n" +
                        "  -tc value         Check CoCos                                Default: true if not tt\n" +
                        "  -tt value         Check whether all tests end with \"Test\"  Default: false\n\n";
    }

    private static ReportContext getContext(String[] args) {
        ReportContext context = new ReportContext();

        if(args.length < 2 || args[0].equals("-h")){
            Log.error(help());
            return null;
        }

        File projectRoot = new File(args[0]);
        File outPut = new File(args[1]);

        if(!projectRoot.isDirectory() || !projectRoot.exists())
            Log.error("Cannot find dir: " + projectRoot.getAbsolutePath());
        if(!outPut.isDirectory() || !outPut.exists())
            Log.error("Cannot find dir: " + outPut.getAbsolutePath());
        context.setProjectRoot(projectRoot.getAbsolutePath());
        context.setOutput(outPut.getAbsolutePath());

        boolean tcSetTrue = false;
        for(int i = 2; i < args.length; i++){
            switch(args[i]){
                case "-tc":
                    try {
                        boolean temp = Boolean.parseBoolean(args[++i]);
                        context.setTestCoCos(temp);
                        tcSetTrue = temp;
                    } catch (NumberFormatException e) {
                        Log.error("Wrong format, see -h for help.");
                    }
                    break;
                case "-tt":
                    try {
                        boolean temp = Boolean.parseBoolean(args[++i]);
                        context.setTestsEndWithTest(temp);
                        if(temp && !tcSetTrue)
                            context.setTestCoCos(false);
                    } catch (NumberFormatException e) {
                        Log.error("Wrong format, see -h for help.");
                    }
                    break;
                case "-h":
                case "-help":
                    System.out.println(help());
                    System.exit(0);
                default:
                    Log.error("Invalid arguments, see -h for help");
            }
        }

        return context;
    }
}
