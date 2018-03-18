package de.monticore.reporting;

import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.reporting.order.OrderTestResults;
import de.monticore.reporting.svgTools.VisualisationHelper;
import de.monticore.reporting.cocoReport.CheckCoCos;
import de.monticore.reporting.testReport.CheckTests;
import de.monticore.reporting.grammarReport.ReportGrammar;
import de.monticore.reporting.cocoReport.helper.*;
import de.monticore.reporting.testReport.TestsTestResultPrinter;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.util.List;

public class Main {
    public static void main(String[] args) {
        LogConfig.init();
        ReportContext context = getContext(args);
        if (context.isTestCoCos()) {
            CheckCoCos tcc = new CheckCoCos();
            System.out.println("\n<================Test CoCos================>\n");
            List<CheckCoCoResult> testResults = tcc.testAllCocos(new File(context.getProjectRoot()), context.getZipName(), "ema", "emam");
            OrderTestResults<CheckCoCoResult> order = new OrderTestResults();
            order.orderTestResults(testResults, new CheckCoCoResultCreator());
            List<CheckCoCoResult> rootModels = order.getRootModels();
            List<CheckCoCoResult> mainPackages = order.getMainPackageModels();

            if (context.isSvg()) {
                System.out.println("\n<==============SVG Generation==============>\n");
                VisualisationHelper.generateSVGs(testResults, rootModels, context.getOutput() + "SVG", context.isMerge());
            }

            System.out.println("\n<============Write Test Results============>\n");
            CoCoTestResultPrinter.printTestResults(mainPackages, context.getOutput() + "data.json", context.isMerge());
            TestInfoPrinter.printInfo(testResults, context.getOutput() + "info.json", context.isMerge());
        }
        if (context.isTestsEndWithTest()) {
            CheckTests tewt = new CheckTests();
            System.out.println("\n<================Test Tests================>\n");
            List<CheckTestResult> testResults = tewt.testTestsEndWithTest(new File(context.getProjectRoot()));
            System.out.println("\n<============Write Test Results============>\n");
            TestsTestResultPrinter.printTestsEndWithTestResults(testResults, context.getOutput() + "dataEWT.json", context.isMerge());
        }
        if (context.isReportGrammar()) {
            System.out.println("\n<==============Grammar Report==============>\n");
            ReportGrammar.reportGrammars(context, context.getOutput() + "dataGrammars.json", context.isMerge());
        }
    }

    public static class ReportContext {
        private boolean testsEndWithTest = false;
        private boolean testCoCos = false;
        private String output = "report/data/";
        private String projectRoot = "";
        private String zipName = "";
        private boolean merge = false;
        private boolean svg = false;
        private boolean reportGrammar = false;
        private String grammarZip = "";

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

        public boolean isReportGrammar() {
            return reportGrammar;
        }

        public void setReportGrammar(boolean reportGrammar) {
            this.reportGrammar = reportGrammar;
        }

        public String getGrammarZip() {
            return grammarZip;
        }

        public void setGrammarZip(String grammarZip) {
            this.grammarZip = grammarZip;
        }
    }

    private static String help() {
        return
                "\nUsage: reporting.jar projectRoot [options]\n\n" +
                        "PARAMETERS\n" +
                        "  projectRoot         The directory with all projects in\n" +
                        "OPTIONS\n" +
                        "  -h --help           Prints this help page\n" +
                        "  -tc                 Test CoCos\n" +
                        "                        If this option is true [default] -zn must be set as well\n" +
                        "    -zn  \"models.zip\"   The zip-file with all models in it\n" +
                        "                        This zip must be hosted on \"github.com/EmbeddedMontiArc/reporting/\"\n" +
                        "                        (Can be a Dummy.zip) \n" +
                        "    -svg                Generate SVG-files (visualisation)          Default: true\n\n" +
                        "  -tt                 Check whether all tests end with \"Test\"   Default: false\n" +
                        "  -grammar            Creates a report for all grammars in the rootFile\n" +
                        "    -gzn \"grammar.zip\"  The zip-file with all grammars in it\n\n" +
                        "  -m                  Merge the output data                       Default: false\n\n";
    }

    private static ReportContext getContext(String[] args) {
        ReportContext context = new ReportContext();

        if (args.length < 1 || args[0].equals("-h")) {
            Log.error(help());
        }

        File projectRoot = new File(args[0]);

        if (!projectRoot.isDirectory() || !projectRoot.exists())
            Log.error("Cannot find dir: " + projectRoot.getAbsolutePath());
        context.setProjectRoot(projectRoot.getAbsolutePath());

        for (int i = 1; i < args.length; i++) {
            switch (args[i]) {
                case "-grammar":
                    context.setReportGrammar(true);
                    break;
                case "-tc":
                    context.setTestCoCos(true);
                    break;
                case "-tt":
                    context.setTestsEndWithTest(true);
                    break;
                case "-m":
                    context.setMerge(true);
                    break;
                case "-svg":
                    context.setSvg(true);
                    break;
                case "-zn":
                    context.setZipName(args[++i]);
                    if (context.getZipName().equals(""))
                        Log.error("Zip name is missing, see -h for help.");
                    break;
                case "-gzn":
                    context.setGrammarZip(args[++i]);
                    if (context.getGrammarZip().equals(""))
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

        if (context.isTestCoCos() && context.getZipName().equals(""))
            Log.error("Invalid arguments, see -h for help");
        if (!context.isTestCoCos() && !context.isTestsEndWithTest() && !context.isReportGrammar())
            Log.error("No options found, see -h for help");
        if (!context.isTestCoCos() && context.isSvg())
            Log.error("Invalid arguments, see -h for help");
        if (context.isReportGrammar() && context.getGrammarZip().equals(""))
            Log.error("Invalid arguments, see -h for help");
        return context;
    }
}
