package de.monticore.reporting.testCocos.helper;

import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.reporting.order.ChildElement;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

public class TestResultPrinter2 {
    
    private static String[] names = {
            "\"Root\"",
            "\"Order\"",
            "\"Project\"",
            "\"ChildExpansion\"",
            "\"ModelName\"",
            "\"Name\"",
            "\"Path\"",
            "\"OnlineIDE\"",
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

    public static void printTestResults(List<CheckCoCoResult> testResults, String path) {
        printTestResults(testResults, path, false);
    }

    public static void printTestResults(List<CheckCoCoResult> testResults, String path, boolean merge) {
        if (testResults.size() == 0) return;
        if (merge) {
            try {
                String first = FileUtils.readFileToString(new File(path));
                first = first.substring(0, first.length() - 3);
                String str = first + ",\n" + printTestResults(testResults, merge, "", null, 0);
                FileUtils.writeStringToFile(new File(path),
                        str);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            try {
                FileUtils.writeStringToFile(new File(path),
                        printTestResults(testResults, merge, "", null, 0));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static String printTestResults(List<CheckCoCoResult> testResults, boolean merge, String baseOrder, File rootFile, int depth) {
        IndentPrinter ip = new IndentPrinter();
        char c = 160;
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
            ip.println(names[i++] + ": \"" + (rootFile == null ? testResult.getRootFile().getName() : rootFile.getName()) + "\",");
            ip.println(names[i++] + ": \"" + getOrder(baseOrder, testResult) + "\",");
            ip.println(names[i++] + ": \"" + testResult.getProject() + "\",");
            ip.println(names[i++] + ": \"" + getDepthImage(testResult, depth) + "\",");
            ip.println(names[i++] + ": \"" + testResult.getModelName() + "\",");
            ip.println(names[i++] + ": \"" + getVisulisationLink(testResult) + "\",");
            ip.println(names[i++] + ": \"" + getFilePath(testResult) + "\",");
            ip.println(names[i++] + ": " + getVFSTag(testResult) + ",");
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
                ip.println(getChildData(testResult, getOrder(baseOrder, testResult), testResult.getRootFile(), depth + 1));
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

    private static String getOrder(String baseOrder, CheckCoCoResult testResult){
        if(testResult.getModelName().equals("Parsing failed")) return CheckCoCoResult.erroredString
                + testResult.getRootFile().getName() + "_Parsing";
        if(testResult.getModelName().equals("Resolving failed")) return CheckCoCoResult.erroredString
                + testResult.getRootFile().getName() + "_Resolving";
        if(testResult.isMainPackage()) return testResult.getRootFile().getName() + "."
                + "." + testResult.getModelName()
                + testResult.getModelPath().substring(testResult.getRootFile().getAbsolutePath().length())
                .replace("\\",".").replace(":",".").replace("/",".")
                ;
        if(baseOrder.equals("")) return testResult.getQualifiedName() + "." + testResult.getProject().replace("/","");
//        String name = testResult.getModelName();
//        if (name.endsWith("ema") || name.endsWith("emam") )
//            name = name.substring(0, name.lastIndexOf("."));
//        if (name.contains("."))
//            name = name.substring(testResult.getModelName().lastIndexOf(".") + 1);
        return baseOrder + "_Child";
    }

    private static String getChildData(CheckCoCoResult testResult, String baseOrder, File rootFile, int depth) {
        List<CheckCoCoResult> childResults = new LinkedList<>();
        for(ChildElement childElement: testResult.getChildren()){
            childResults.add(childElement.getChild());
        }
        return printTestResults(childResults, false, baseOrder + ".", rootFile, depth);
    }

    private static String getDepthImage(CheckCoCoResult testResult, int depth){
        if(testResult.getChildren().size() == 0 || depth == 1) return "<div class=\'atomicImage" + depth + "\'></div>";
        return "<div class=\'depthImage" + depth + "\'></div>";
    }

    private static String getVFSTag(CheckCoCoResult testResult) {
        if(testResult.isErrorResult() || testResult.isMainPackage()) return tagOf(0);
        String zipName = testResult.getZipName();
        File file = testResult.getModelFile();
        File project = testResult.getProjectFile();
        String urlToZip;
        String zipName_;
        if (zipName == null)
            zipName_ = "models1a6a7c6e450b6d996a79c701efdd4e69.zip";
        else
            zipName_ = zipName;

        urlToZip = "https://raw.githubusercontent.com/EmbeddedMontiArc/reporting/gh-pages/" + zipName_;
        zipName_ = zipName_.substring(0, zipName_.lastIndexOf("."));
        String name = file.getAbsolutePath().substring(project.getAbsolutePath().length() - project.getName().length());
        String displayName = name;
//        if (project.getAbsolutePath().contains("MontiSim"))
//            displayName = "MontiSim/" + displayName;
        return ("\"<a target='_blank' href='onlineIDE/api/load.html?mountPoint=EmbeddedMontiArc/reporting/" + zipName_ + "&url="
                + urlToZip + "&openFile=/" + name + "'>" +
                "<img border='0' alt='" + displayName + "' src='images/favicon.ico' class='onlineIDEImage'>" +
                "</a>\"").replace("\\", "/");
    }

    private static String getVisulisationLink(CheckCoCoResult testResult) {
        if(testResult.isErrorResult() || testResult.isMainPackage()) return testResult.getModelName() + " (" + testResult.getChildren().size() + ")";
        File file = testResult.getModelFile();
        String displayName = testResult.getModelName();

        if (testResult.getSvgPath().equals(""))
            return "<div class='noSVG'>" + displayName + "</div>";
        else
            return "<a class='sVG' target='_blank' href='" +
                    testResult.getSvgPath() + "'>" +
                    displayName +
                    "</a>";
    }

    private static String getFilePath(CheckCoCoResult testResult) {
        if(testResult.isErrorResult()) return testResult.getModelName();
        if(testResult.isMainPackage()) return testResult.getRootFile().getName() + "/" + testResult.getProject();
        File file = testResult.getModelFile();
        File project = testResult.getProjectFile();
        String name = file.getAbsolutePath().substring(project.getAbsolutePath().length() - project.getName().length());
        String displayName = name.replace("\\","/");
        return displayName;
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
