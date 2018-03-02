package de.monticore.reporting.testCocos;

import de.monticore.reporting.order.OrderTestResults;
import de.monticore.reporting.svgTools.VisualisationHelper;
import de.monticore.reporting.testCocos.helper.CheckCoCoResult;
import de.monticore.reporting.testCocos.helper.TestResultPrinter;
import de.monticore.reporting.testCocos.helper.CheckTestResult;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.util.List;

public class Main {
    public static void main(String[] args) {
        ReportContext context = getContext(args);
        if (context.isTestCoCos()) {
            CheckCoCos tcc = new CheckCoCos();
            System.out.println("\n<================Test CoCos================>\n");
            List<CheckCoCoResult> testResults = tcc.testAllCocos(new File(context.getProjectRoot()), context.getZipName(), "ema", "emam");
            List<CheckCoCoResult> rootModels = OrderTestResults.orderTestResults(testResults);

            if (context.isSvg()) {
                System.out.println("\n<==============SVG Generation==============>\n");
                VisualisationHelper.generateSVGs(testResults, rootModels, context.getOutput() + "SVG", context.isMerge());
            }

            System.out.println("\n<============Write Test Results============>\n");
            TestResultPrinter.printTestResults(testResults, context.getOutput() + "data.json", context.isMerge());
        }
        if (context.isTestsEndWithTest()) {
            CheckTests tewt = new CheckTests();
            System.out.println("\n<================Test Tests================>\n");
            List<CheckTestResult> testResults = tewt.testTestsEndWithTest(new File(context.getProjectRoot()));
            System.out.println("\n<============Write Test Results============>\n");
            TestResultPrinter.printTestsEndWithTestResults(testResults, context.getOutput() + "/dataEWT.json", context.isMerge());
        }
    }

    protected static class ReportContext {
        private boolean testsEndWithTest = false;
        private boolean testCoCos = true;
        private String output = "report/data/";
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
                "\nUsage: reporting.jar projectRoot [options]\n\n" +
                        "PARAMETERS\n" +
                        "  projectRoot         The directory with all projects in\n" +
                        "OPTIONS\n" +
                        "  -h --help           Prints this help page\n" +
                        "  -tc  [true/false]   Test CoCos                                  Default: true if !tt\n" +
                        "                        If this option is true [default] -zn must be set as well\n" +
                        "  -zn  value          The zip-file with all models in it\n" +
                        "                        This zip must be hosted on \"github.com/EmbeddedMontiArc/reporting/\"\n" +
                        "  -svg [true/false]   Generate SVG-files (visualisation)          Default: true\n" +
                        "  -tt  [true/false]   Check whether all tests end with \"Test\"   Default: false\n" +
                        "  -m   [true/false]   Merge the output data                       Default: false\n\n";
    }

    private static ReportContext getContext(String[] args) {
        ReportContext context = new ReportContext();

        if (args.length < 1 || args[0].equals("-h")) {
            Log.error(help());
            return null;
        }

        File projectRoot = new File(args[0]);

        if (!projectRoot.isDirectory() || !projectRoot.exists())
            Log.error("Cannot find dir: " + projectRoot.getAbsolutePath());
        context.setProjectRoot(projectRoot.getAbsolutePath());

        boolean tcSetTrue = false;
        for (int i = 1; i < args.length; i++) {
            switch (args[i]) {
                case "-tc":
                    try {
                        String c = args[++i];
                        try {
                            boolean temp = Boolean.parseBoolean(c);
                            context.setTestCoCos(temp);
                            tcSetTrue = temp;
                        } catch (Exception e) {
                            Log.error("Could not parse \"" + c + "\", see -h for help.");
                        }
                    } catch (Exception e) {
                        Log.error("Missing argument for option \"tc\", see -h for help.");
                    }
                    break;
                case "-tt":
                    try {
                        String c = args[++i];
                        try {
                            boolean temp = Boolean.parseBoolean(c);
                            context.setTestsEndWithTest(temp);
                            if (temp && !tcSetTrue)
                                context.setTestCoCos(false);
                        } catch (Exception e) {
                            Log.error("Could not parse \"" + c + "\", see -h for help.");
                        }
                    } catch (Exception e) {
                        Log.error("Missing argument for option \"tt\", see -h for help.");
                    }
                    break;
                case "-m":
                    try {
                        String c = args[++i];
                        try {
                            boolean temp = Boolean.parseBoolean(c);
                            context.setMerge(temp);
                        } catch (NumberFormatException e) {
                            Log.error("Could not parse \"" + c + "\", see -h for help.");
                        }
                    } catch (Exception e) {
                        Log.error("Missing argument for option \"m\", see -h for help.");
                    }
                    break;
                case "-svg":
                    try {
                        String c = args[++i];
                        try {
                            boolean temp = Boolean.parseBoolean(c);
                            context.setSvg(temp);
                        } catch (NumberFormatException e) {
                            Log.error("Could not parse \"" + c + "\", see -h for help.");
                        }
                    } catch (Exception e) {
                        Log.error("Missing argument for option \"svg\", see -h for help.");
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
