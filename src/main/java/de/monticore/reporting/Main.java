/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting;

import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.reporting.order.OrderTestResults;
import de.monticore.reporting.cocoReport.CheckCoCos;
import de.monticore.reporting.testReport.CheckTests;
import de.monticore.reporting.grammarReport.ReportGrammar;
import de.monticore.reporting.cocoReport.helper.*;
import de.monticore.reporting.testReport.TestsTestResultPrinter;
import de.monticore.reporting.tools.CustomPrinter;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

public class Main {
    public static void main(String[] args) {
        LogConfig.init();
        Log.init();
        CustomPrinter.init();
        ReportContext context = getContext(args);
        if (context.isTestCoCos()) {
            ManageTesting.manageTesting(context);
        }
        if (context.isTestsEndWithTest()) {
            CheckTests tewt = new CheckTests();
            CustomPrinter.println("\n<================Test Tests================>\n");
            List<CheckTestResult> testResults = tewt.testTestsEndWithTest(new File(context.getProjectRoot()));
            CustomPrinter.println("SUCCESS\n");
            CustomPrinter.println("\n<============Write Test Results============>\n");
            TestsTestResultPrinter.printTestsEndWithTestResults(testResults, context.getOutput() + "dataEWT.json", context.isMerge());
            CustomPrinter.println("SUCCESS\n");
        }
        if (context.isReportGrammar()) {
            CustomPrinter.println("\n<==============Grammar Report==============>\n");
            ReportGrammar.reportGrammars(context, context.getOutput() + "dataGrammars.json", context.isMerge());
            CustomPrinter.println("SUCCESS\n");
        }

        CustomPrinter.println("\n<===========Copy Data Repository===========>\n");
        File dataDir = new File("report/data");
        if (dataDir.exists()) {
            try {
                FileUtils.deleteDirectory(dataDir);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        dataDir.mkdir();
        try {
            FileUtils.copyDirectory(new File(context.getOutput()), dataDir);
        } catch (IOException e) {
            e.printStackTrace();
        }
        CustomPrinter.println("SUCCESS\n");
        System.exit(0);
    }

    public static class ReportContext {
        private boolean testsEndWithTest = false;
        private boolean testCoCos = false;
        private String projectRoot = "";
        private boolean merge = false;
        private boolean reportGrammar = false;
        private Date date = Calendar.getInstance().getTime();
        private String output = "data/data_" + (new SimpleDateFormat("yyyyMMdd_HHmmss").format(date)) + "/";
        private int timeout = 10;
        private int coCoTimeOut = 1;

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

        public boolean isMerge() {
            return merge;
        }

        public void setMerge(boolean merge) {
            this.merge = merge;
        }

        public boolean isReportGrammar() {
            return reportGrammar;
        }

        public void setReportGrammar(boolean reportGrammar) {
            this.reportGrammar = reportGrammar;
        }

        public int getTimeout() {
            return timeout;
        }

        public void setTimeout(int timeout) {
            this.timeout = timeout;
        }

        public Date getDate() {
            return date;
        }

        public void setDate(Date date) {
            this.date = date;
        }

        public int getCoCoTimeOut() {
            return coCoTimeOut;
        }

        public void setCoCoTimeOut(int coCoTimeOut) {
            this.coCoTimeOut = coCoTimeOut;
        }
    }

    private static String help() {
        return
                "\nUsage: reporting.jar projectRoot [options]\n\n" +
                        "PARAMETERS\n" +
                        "  projectRoot         The directory with all projects in\n" +
                        "OPTIONS\n" +
                        "  -h --help           Prints this help page\n" +
                        "  -testCoCos          Test CoCos                                  Default: not set\n" +
                        "     -timeout \"t\"       Set timeout for symtab building           Default: 10s\n" +
                        "     -cTimeout \"t\"      Set timeout for CoCo Checking             Default: timeout / 5 s\n" +
                        "  -testTests          Check whether all tests end with \"Test\"   Default: not set\n" +
                        "  -grammar            Creates a report for all grammars in the directory\n" +
                        "  -out \"directory\"    Output directory                            Default: report/data[CurrentTime]/\n" +
                        "  -m                  Merge the output data                       Default: not set\n\n";
    }

    private static ReportContext getContext(String[] args) {
        ReportContext context = new ReportContext();

        if (args.length < 1 || args[0].equals("-h")) {
            Log.error(help());
        }

        File projectRoot = new File(args[0]);
        if (!projectRoot.isDirectory() || !projectRoot.exists()) {
            CustomPrinter.println("Cannot find dir: " + projectRoot.getAbsolutePath());
            Log.error("Cannot find dir: " + projectRoot.getAbsolutePath());
        }
        context.setProjectRoot(projectRoot.getAbsolutePath());

        boolean coCoTimeOutSet = false;
        try {
            for (int i = 1; i < args.length; i++) {
                switch (args[i]) {
                    case "-grammar":
                        context.setReportGrammar(true);
                        break;
                    case "-testCoCos":
                        context.setTestCoCos(true);
                        break;
                    case "-testTests":
                        context.setTestsEndWithTest(true);
                        break;
                    case "-m":
                        context.setMerge(true);
                        break;
                    case "-out":
                        context.setOutput(args[++i]);
                        break;
                    case "-h":
                    case "--help":
                        CustomPrinter.println(help());
                        System.exit(0);
                    case "-timeout":
                        int t = Integer.parseInt(args[++i]);
                        context.setTimeout(t);
                        if (!coCoTimeOutSet && t / 5 >= 1)
                            context.setCoCoTimeOut(context.getTimeout() / 5);
                        else if (!coCoTimeOutSet && t >= 2)
                            context.setCoCoTimeOut(2);
                        else if (!coCoTimeOutSet)
                            context.setCoCoTimeOut(1);
                        break;
                    case "-cTimeout":
                        context.setCoCoTimeOut(Integer.parseInt(args[++i]));
                        break;
                    default:
                        CustomPrinter.println("Invalid arguments for \"" + args[i] + "\":\n" + help());
                        System.exit(0);
                }
            }
        } catch (Throwable e) {
            CustomPrinter.println("Invalid arguments\n" + help());
            System.exit(0);
        }

        if (!context.isTestCoCos() && !context.isTestsEndWithTest() && !context.isReportGrammar())
            Log.error("No options found, see -h for help");

        File outDir = new File(context.getOutput());
        if (outDir.exists() && !outDir.isDirectory())
            Log.error("Output " + outDir.getAbsolutePath() + " already is a file");
        if(!outDir.exists())
            outDir.mkdirs();
        context.setOutput(outDir.getAbsolutePath());

        return context;
    }
}
