/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

import de.monticore.reporting.helper.FilePrinter;
import de.monticore.reporting.order.ChildElement;
import de.monticore.reporting.tools.CustomPrinter;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

public class CoCoTestResultPrinter {

    private static int progress = 0;
    private static int total = 0;
    private static int z = 0;
    private static FilePrinter ip;
    private static boolean printProgress = true;

    private static String[] names = {
            "\"Root\"",
            "\"Project\"",
            "\"ChildExpansion\"",
            "\"ModelName\"",
            "\"Name\"",
            "\"Path\"",
//            "\"Visualisation\"",
//            "\"OnlineIDE\"",
            "\"LogNr\"",
            "\"LogOutput\"",
            "\"FileType\"",
            "\"Valid\"",
            "\"Parse\"",
            "\"Resolve\"",
            "\"ComponentCapitalized\"",
            "\"ComponentInstanceNamesUnique\"",
            "\"ComponentWithTypeParametersHasInstance\"",
            "\"ConnectorEndPointCorrectlyQualified\"",
            "\"DefaultParametersHaveCorrectOrder\"",
            "\"InPortUniqueSender\"",
            "\"PackageLowerCase\"",
            "\"ParameterNamesUnique\"",
            "\"PortTypeOnlyBooleanOrSIUnit\"",
            "\"PortUsage\"",
            "\"ReferencedSubComponentExists\"",
            "\"SimpleConnectorSourceExists\"",
            "\"SourceTargetNumberMatch\"",
            "\"SubComponentsConnected\"",
            "\"TopLevelComponentHasNoInstanceName\"",
            "\"TypeParameterNamesUnique\"",
            "\"AtomicComponent\"",
            "\"UniquePorts\"",
            "\"ChildData\""
    };
    private static String tickTag = "\"<img src='images/tick.png'/>\"";
    private static String crossTag = "\"<img src='images/Red_cross_tick.png'/>\"";
    private static String noTag = "\"<img src='images/minus.jpg'/>\"";
    private static String timeTag = "\"<img src='images/timeout.png'/>\"";

    private static String tagOf(int i) {
        switch (i) {
            case -2:
                return timeTag;
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

    public static void printTestResults(List<CheckCoCoResult> testResults, String path, boolean merge, boolean group, boolean print) {
        if (!(new File(path)).exists())
            merge = false;

        if (testResults.size() == 0) return;
        int depth = group ? 0 : 1;
        z = 0;
        printProgress = print;
        if (printProgress) {
            progress = 0;
            total = calcTotal(testResults, group);
            for (int j = 0; j < 50; j++)
                CustomPrinter.print("|");
            CustomPrinter.println("");
        }

        if (merge) {
            try {
                String str = FileUtils.readFileToString(new File(path));
                str = str.substring(0, str.length() - 2);
                FileUtils.writeStringToFile(new File(path),
                        str + ",");
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            File file = new File(path);
            if (file.exists())
                file.delete();
        }
        ip = new FilePrinter(path);
        printTestResults(testResults, merge, "", depth, !group);
        ip.end();

        CustomPrinter.println("");
        CustomPrinter.println("");
    }

    public static void printTestResults(List<CheckCoCoResult> testResults, boolean merge, String rootName, int depth, boolean expanded) {
        if (!merge)
            ip.println("[");
        ip.indent();

        boolean first = true;
        for (CheckCoCoResult testResult : testResults) {
            if (testResult == null) continue;
            if (depth == 1)
                z++;
            int i = 0;

            if (!first)
                ip.println(",");
            else
                first = false;

            ip.println("{");
            ip.indent();
            if (depth == 1 && !rootName.equals(testResult.getRootName())){
                int r = 1;
            }
            ip.println("\"Root\"" + ": \"" + (rootName.equals("") ? testResult.getRootName() : rootName) + "\",");
            ip.println("\"Project\"" + ": \"" + testResult.getProject() + "\",");
            ip.println("\"ChildExpansion\"" + ": \"" + getDepthImage(testResult, depth) + "\",");
            ip.println("\"ModelName\"" + ": \"" + testResult.getModelName() + "\",");
            ip.println("\"Name\"" + ": \"" + getNameWithGitLabLink(testResult, expanded) + "\",");
            ip.println("\"Path\"" + ": \"" + getFilePath(testResult) + "\",");
            ip.println("\"LogNr\"" + ": \"" + testResult.getErrorMessages().size() + "\",");
            ip.println("\"LogOutput\"" + ": \"" + testResult.getErrorMessage() + "\",");
            ip.println("\"FileType\"" + ": \"" + testResult.getFileType() + "\",");
            ip.println("\"Valid\"" + ": " + tagOf(testResult.isValid() ? 1 : -1) + ",");
            ip.println("\"Parse\"" + ": " + tagOf(testResult.getParsed()) + ",");
            ip.println("\"Resolve\"" + ": " + tagOf(testResult.getResolved()) + ",");

            CheckCoCoResult.Data[] data = testResult.getFieldsAndValues();
            for (CheckCoCoResult.Data dat : data)
                ip.println(dat.getQuotedName() + ": " + tagOf(dat.value) + ",");

            ip.println("\"ChildData\"" + ": ");
            if (depth == 0)
                printChildData(testResult, testResult.getRootName(), depth + 1);
            else
                ip.println("[]");
            ip.unindent();
            ip.print("}");

            if (depth == 1 && printProgress)
                printProgress();
        }
        ip.println("");
        ip.unindent();
        ip.println("]");
    }

    private static int calcTotal(List<CheckCoCoResult> testResults, boolean group) {
        if (!group) return testResults.size();
        int i = 0;
        for (CheckCoCoResult tr : testResults)
            i += tr.getChildren().size();
        return i;
    }

    private static void printProgress() {
        int currentProgress = (z * 50 / total);
        if (currentProgress > progress) {
            for (int j = 0; j < currentProgress - progress; j++)
                CustomPrinter.print("|");
            progress = currentProgress;
        }
    }

    private static void printChildData(CheckCoCoResult testResult, String rootName, int depth) {
        List<CheckCoCoResult> childResults = new LinkedList<>();
        for (ChildElement childElement : testResult.getChildren()) {
            childResults.add((CheckCoCoResult) childElement.getChild());
        }
        printTestResults(childResults, false, rootName, depth, false);
    }

    private static String getDepthImage(CheckCoCoResult testResult, int depth) {
        if (testResult.getChildren().size() == 0 || depth == 1) return "<div class=\'atomicImage" + depth + "\'></div>";
        return "<div class=\'depthImage" + depth + "\'></div>";
    }

    private static String getNameWithGitLabLink(CheckCoCoResult testResult, boolean expanded) {
        if (testResult.isErrorResult() || testResult.isMainPackage())
            return testResult.getModelName() + " (" + testResult.getChildren().size() + ")";
        String gLLink = testResult.getGitLabLink();

        String displayName = testResult.getModelName();
        if (expanded)
            displayName = testResult.getProject() + displayName;

        String htmlTag = "<a class='ghLink' href='" + gLLink + "' target='_blank' rel='noopener'>"
                + displayName + "</a>";


        return htmlTag;
    }

    private static String getFilePath(CheckCoCoResult testResult) {
        if (testResult.isErrorResult()) return testResult.getModelName();
        if (testResult.isMainPackage()) return testResult.getRootName() + "/" + testResult.getProject();
        File file = testResult.getModelFile();
        String name = file.getAbsolutePath().substring(testResult.getRootFile().getAbsolutePath().length());
        String displayName = name.replace("\\", "/");
        return displayName;
    }
}
