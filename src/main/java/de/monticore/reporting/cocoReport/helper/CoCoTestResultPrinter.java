package de.monticore.reporting.cocoReport.helper;

import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.reporting.order.ChildElement;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

public class CoCoTestResultPrinter {
    
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

    public static void printTestResults(List<CheckCoCoResult> testResults, String path, boolean merge, boolean group) {
        if (testResults.size() == 0) return;
        int depth = group ? 0 : 1;
        if (merge) {
            try {
                String first = FileUtils.readFileToString(new File(path));
                first = first.substring(0, first.length() - 3);
                String str = first + ",\n" + printTestResults(testResults, merge, "", depth, !group);
                FileUtils.writeStringToFile(new File(path),
                        str);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            try {
                FileUtils.writeStringToFile(new File(path),
                        printTestResults(testResults, merge, "", depth, !group));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static String printTestResults(List<CheckCoCoResult> testResults, boolean merge, String rootName, int depth, boolean expanded) {
        IndentPrinter ip = new IndentPrinter();
        if (!merge)
            ip.println("[");
        ip.indent();

        boolean first = true;
        for (CheckCoCoResult testResult : testResults) {
            if (testResult == null) continue;
            int i = 0;

            if (!first)
                ip.print(",\n");
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
            ip.indent();
            if(depth == 0)
                ip.println(getChildData(testResult, testResult.getRootName1(), depth + 1));
            else
                ip.println("[]");
            ip.unindent();
            ip.unindent();
            ip.print("}");
        }
        ip.println();
        ip.unindent();
        ip.println("]");

        return ip.getContent();
    }

    private static String getChildData(CheckCoCoResult testResult, String rootName, int depth) {
        List<CheckCoCoResult> childResults = new LinkedList<>();
        for(ChildElement childElement: testResult.getChildren()){
            childResults.add((CheckCoCoResult) childElement.getChild());
        }
        return printTestResults(childResults, false, rootName, depth, false);
    }

    private static String getDepthImage(CheckCoCoResult testResult, int depth){
        if(testResult.getChildren().size() == 0 || depth == 1) return "<div class=\'atomicImage" + depth + "\'></div>";
        return "<div class=\'depthImage" + depth + "\'></div>";
    }

//    private static String getVFSTag(CheckCoCoResult testResult) {
//        if(testResult.isErrorResult() || testResult.isMainPackage()) return tagOf(0);
//        String zipName = testResult.getZipName();
//        File file = testResult.getModelFile();
//        File root = testResult.getRootFile1();
//        String urlToZip;
//
//        urlToZip = "https://raw.githubusercontent.com/EmbeddedMontiArc/reporting/gh-pages/" + zipName;
//        zipName = zipName.substring(0, zipName.lastIndexOf("."));
//        String name = file.getAbsolutePath().substring(root.getAbsolutePath().length());
//        String displayName = name;
//        return ("\"<a class='ideLink' target='_blank' rel='noopener' href='onlineIDE/api/load.html?mountPoint=EmbeddedMontiArc/reporting/" + zipName + "&url="
//                + urlToZip + "&openFile=/" + name + "'>" +
//                "<img border='0' alt='" + displayName + "' src='images/favicon.ico' class='onlineIDEImage'>" +
//                "</a>\"").replace("\\", "/");
//    }
//
//    private static String getVisulisationLink(CheckCoCoResult testResult) {
//        String displayName = testResult.getModelName();
//
//
//        if (testResult.isErrorResult() || testResult.isMainPackage() || testResult.isAtomic())
//            return "\"<div class='svgMinus'>" +
//                    "<img alt='" + displayName + "' src='images/minus.jpg'>" +
//                    "</div>\"";
//        else if (testResult.getSvgPath().equals(""))
//            return "\"<div class='svgNoEye'>" +
//                    "<img alt='" + displayName + "' src='images/noEye.png'>" +
//                    "</div>\"";
//        else
//            return "\"<a class='svgEye' target='_blank' rel='noopener' href='" +
//                    testResult.getSvgPath() + "'>" +
//                    "<img alt='" + displayName + "' src='images/eye.png'>" +
//                    "</a>\"";
//    }

    private static String getNameWithGitLabLink(CheckCoCoResult testResult, boolean expanded) {
        if(testResult.isErrorResult() || testResult.isMainPackage()) return testResult.getModelName() + " (" + testResult.getChildren().size() + ")";
        String gLLink = testResult.getGitLabLink();

        String displayName = testResult.getModelName();
        if (expanded)
            displayName = testResult.getProject() + displayName;

        String htmlTag = "<a class='ghLink' href='" + gLLink + "' target='_blank' rel='noopener'>"
                + displayName + "</a>";


        return htmlTag;
    }

    private static String getFilePath(CheckCoCoResult testResult) {
        if(testResult.isErrorResult()) return testResult.getModelName();
        if(testResult.isMainPackage()) return testResult.getRootName1()+ "/" + testResult.getProject();
        File file = testResult.getModelFile();
        String name = file.getAbsolutePath().substring(testResult.getRootFile1().getAbsolutePath().length());
        String displayName = name.replace("\\","/");
        return displayName;
    }
}
