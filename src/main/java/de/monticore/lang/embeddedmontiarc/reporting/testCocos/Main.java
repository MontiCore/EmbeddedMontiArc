package de.monticore.lang.embeddedmontiarc.reporting.testCocos;

import de.monticore.lang.embeddedmontiarc.reporting.helper.VisualisationHelper;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResultPrinter;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestsEndWithTestResult;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.util.List;

public class Main {
    public static void main(String[] args) {
        ReportContext context = getContext(args);
        if (context.isTestCoCos()) {
            TestCoCos tcc = new TestCoCos();
            List<TestResult> testResults = tcc.testAllCocos(new File(context.getProjectRoot()), context.getZipName(), "ema", "emam");
            if (context.isSvg()) {
//                VisualisationHelper.init("C:/Praktikum/rep2/reporting/EmbeddedMontiArc/EMAM2Cpp/src/test/resources/", "testing.componentArray");
                VisualisationHelper.generateSVGs(testResults, context.getOutput() + "SVG");
            }
            TestResultPrinter.printTestResults(testResults, context.getOutput() + "data.json", context.isMerge());
        }
        if (context.isTestsEndWithTest()) {
            TestsEndWithTest tewt = new TestsEndWithTest();
            List<TestsEndWithTestResult> testResults = tewt.testTestsEndWithTest(new File(context.getProjectRoot()));
            TestResultPrinter.printTestsEndWithTestResults(testResults, context.getOutput() + "/dataEWT.json", context.isMerge());
        }
    }

    protected static class ReportContext {
        private boolean testsEndWithTest = false;
        private boolean testCoCos = true;
        private String output = "";
        private String projectRoot = "";
        private String zipName = "";
        private boolean merge = false;
        private boolean svg = true;

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
            String res = output.replace("\\", "/");
            if (res.charAt(res.length() - 1) != '/')
                res = res + "/";
            this.output = res;
        }

        public String getProjectRoot() {
            return projectRoot;
        }

        public void setProjectRoot(String projectRoot) {
            this.projectRoot = projectRoot;
        }

        public String getZipName() {
            return zipName;
        }

        public void setZipName(String zipName) {
            this.zipName = zipName;
        }

        public boolean isMerge() {
            return merge;
        }

        public void setMerge(boolean merge) {
            this.merge = merge;
        }

        public boolean isSvg() {
            return svg;
        }

        public void setSvg(boolean svg) {
            this.svg = svg;
        }
    }

    private static String help() {
        return
                "\n\nUsage: reporting projectRoot outputPath [options]\n\n" +
                        "PARAMETERS\n" +
                        "  projectRoot         Points to the directory with all projects in\n" +
                        "  outputPath          Points to the directory with the html file in\n" +
                        "OPTIONS\n" +
                        "  -h --help           Prints this help page\n" +
                        "  -tc  [true/false]   Check CoCos                                 Default: true if not tt\n" +
                        "                        if this option is true [default] -zn must be set" +
                        "  -zn  value          The name (containing file ending) of the zip-file with all models in it\n" +
                        "  -svg [true/false]   Generate SVG-files (visualisation)          Default: true\n" +
                        "  -tt  [true/false]   Check whether all tests end with \"Test\"   Default: false\n" +
                        "  -m   [true/false]   Merge the output data                       Default: false\n\n";
    }

    private static ReportContext getContext(String[] args) {
        ReportContext context = new ReportContext();

        if (args.length < 2 || args[0].equals("-h")) {
            Log.error(help());
            return null;
        }

        File projectRoot = new File(args[0]);
        File outPut = new File(args[1]);

        if (!projectRoot.isDirectory() || !projectRoot.exists())
            Log.error("Cannot find dir: " + projectRoot.getAbsolutePath());
        if (!outPut.isDirectory() || !outPut.exists())
            Log.error("Cannot find dir: " + outPut.getAbsolutePath());
        context.setProjectRoot(projectRoot.getAbsolutePath());
        context.setOutput(outPut.getAbsolutePath());

        boolean tcSetTrue = false;
        for (int i = 2; i < args.length; i++) {
            switch (args[i]) {
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
                        if (temp && !tcSetTrue)
                            context.setTestCoCos(false);
                    } catch (NumberFormatException e) {
                        Log.error("Wrong format, see -h for help.");
                    }
                    break;
                case "-m":
                    try {
                        boolean temp = Boolean.parseBoolean(args[++i]);
                        context.setMerge(temp);
                    } catch (NumberFormatException e) {
                        Log.error("Wrong format, see -h for help.");
                    }
                    break;
                case "-svg":
                    try {
                        boolean temp = Boolean.parseBoolean(args[++i]);
                        context.setSvg(temp);
                    } catch (NumberFormatException e) {
                        Log.error("Wrong format, see -h for help.");
                    }
                    break;
                case "-zn":
                    context.setZipName(args[++i]);
                    if (context.getZipName().equals(""))
                        Log.error("Zip name is missing, see -h for help.");
                    break;
                case "-h":
                case "-help":
                    System.out.println(help());
                    System.exit(0);
                default:
                    Log.error("Invalid arguments, see -h for help");
            }
        }

        if (context.isTestCoCos() && context.getZipName().equals("")) {
            Log.error("Invalid arguments, see -h for help");
        }

        return context;
    }
}
