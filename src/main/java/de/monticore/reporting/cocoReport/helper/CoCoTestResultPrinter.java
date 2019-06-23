package de.monticore.reporting.cocoReport.helper;

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

    public static void printTestResults(List<CheckCoCoResult> testResults, String path, boolean merge, boolean group) {
        if (testResults.size() == 0) return;
        int depth = group ? 0 : 1;
        z = 0;
        progress = 0;
        total = calcTotal(testResults, group);
        for (int j = 0; j < 50; j++)
            CustomPrinter.print("|");
        CustomPrinter.println("");

        if (merge) {
            try {
                String str = FileUtils.readFileToString(new File(path));
                str = str.substring(0, str.length() - 3);
                FileUtils.writeStringToFile(new File(path),
                        str);
                printTestResults(testResults, merge, "", depth, !group);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            File file = new File(path);
            if (file.exists())
                file.delete();
            ip = new FilePrinter(path);
            printTestResults(testResults, merge, "", depth, !group);
        }

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
            ip.println(names[i++] + ": \"" + (rootName.equals("") ? testResult.getRootName1() : rootName) + "\",");
            ip.println(names[i++] + ": \"" + testResult.getProject() + "\",");
            ip.println(names[i++] + ": \"" + getDepthImage(testResult, depth) + "\",");
            ip.println(names[i++] + ": \"" + testResult.getModelName() + "\",");
            ip.println(names[i++] + ": \"" + getNameWithGitLabLink(testResult, expanded) + "\",");
            ip.println(names[i++] + ": \"" + getFilePath(testResult) + "\",");
//            ip.println(names[i++] + ": " + getVisulisationLink(testResult) + ",");
//            ip.println(names[i++] + ": " + getVFSTag(testResult) + ",");
            ip.println(names[i++] + ": \"" + testResult.getErrorMessages().size() + "\",");
            ip.println(names[i++] + ": \"" + testResult.getErrorMessage() + "\",");
            ip.println(names[i++] + ": \"" + testResult.getFileType() + "\",");
            ip.println(names[i++] + ": " + tagOf(testResult.isValid() ? 1 : -1) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getParsed()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getResolved()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getComponentCapitalized()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getComponentInstanceNamesUnique()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getComponentWithTypeParametersHasInstance()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getConnectorEndPointCorrectlyQualified()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getDefaultParametersHaveCorrectOrder()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getInPortUniqueSender()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getPackageLowerCase()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getParameterNamesUnique()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getPortTypeOnlyBooleanOrSIUnit()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getPortUsage()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getReferencedSubComponentExists()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getSimpleConnectorSourceExists()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getSourceTargetNumberMatch()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getSubComponentsConnected()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getTopLevelComponentHasNoInstanceName()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getTypeParameterNamesUnique()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getAtomicComponent()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getUniquePorts()) + ",");
            ip.println(names[i++] + ": ");
            if (depth == 0)
                printChildData(testResult, testResult.getRootName1(), depth + 1);
            else
                ip.println("[]");
            ip.unindent();
            ip.print("}");

            if (depth == 1)
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
        if (testResult.isMainPackage()) return testResult.getRootName1() + "/" + testResult.getProject();
        File file = testResult.getModelFile();
        String name = file.getAbsolutePath().substring(testResult.getRootFile1().getAbsolutePath().length());
        String displayName = name.replace("\\", "/");
        return displayName;
    }
}
