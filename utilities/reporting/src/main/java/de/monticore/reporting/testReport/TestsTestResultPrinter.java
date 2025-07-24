/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.testReport;

import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.reporting.cocoReport.helper.CheckTestResult;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class TestsTestResultPrinter {

    private static String tickTag = "\"<img src='images/tick.png'/>\"";
    private static String crossTag = "\"<img src='images/Red_cross_tick.png'/>\"";
    private static String noTag = "\"<img src='images/minus_318-140716.jpg'/>\"";

    private static String tagOf(int i) {
        switch (i) {
            case -1:
                return crossTag;
            case 0:
                return noTag;
            case 1:
                return tickTag;
            default:
                return noTag;
        }
    }

    public static void printTestsEndWithTestResults(List<CheckTestResult> results, String path) {
        printTestsEndWithTestResults(results, path, false);
    }

    public static void printTestsEndWithTestResults(List<CheckTestResult> results, String path, boolean merge) {
        if (results.size() == 0) return;
        if (merge) {
            try {
                String first = FileUtils.readFileToString(new File(path));
                first = first.substring(0, first.length() - 2);
                String str = first + ",\n" + printTestsEndWithTestResults(results, merge);
                FileUtils.writeStringToFile(new File(path),
                        str);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else
            try {
                FileUtils.writeStringToFile(new File(path), printTestsEndWithTestResults(results, merge));
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    public static String printTestsEndWithTestResults(List<CheckTestResult> results, boolean merge) {
        IndentPrinter ip = new IndentPrinter();
        if (!merge)
            ip.println("[");
        ip.indent();

        boolean first = true;
        for (CheckTestResult testResult : results) {
            if (testResult == null /*testResult.isValid()*/) continue;

            if (!first)
                ip.print(",\n");
            else
                first = false;

            ip.println("{");
            ip.indent();
            ip.println("\"Path\": \"" + testResult.getPath().replace("\\", "/") + "\",");
            ip.println("\"NameEndsWithTest\": " + tagOf(testResult.endsWithTest() ? 1 : -1));
            ip.unindent();
            ip.print("}");
        }
        ip.println();
        ip.unindent();
        ip.println("]");

        return ip.getContent();
    }
}
