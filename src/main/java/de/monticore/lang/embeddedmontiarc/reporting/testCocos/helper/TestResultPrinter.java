package de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper;

import de.monticore.lang.monticar.helper.IndentPrinter;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class TestResultPrinter {

    private static String[] names = {
            "\"ModelName\"",
            "\"Name\"",
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
            "\"AtomicComponent\""
    };
    private static String tickTag = "\"<img src='images/tick.png'/>\"";
    private static String crossTag = "\"<img src='images/Red_cross_tick.png'/>\"";
    private static String noTag = "\"<img src='images/minus_318-140716.jpg'/>\"";

    private static String tagOf(int i){
        switch(i){
            case -1: return crossTag;
            case 0: return noTag;
            case 1: return tickTag;
            default: return noTag;
        }
    }

    private static String modelName_hiddenPath(String modelName, String path){
        path = path.replace("\\","/");
        String name = modelName;
        if (path.contains("MontiSim")){
            name = "MontiSim/" + name;
        }
        return
                "<div class=\'shortLabel\'>" + name + "</div>" +
                "<div class=\'fullLabel\' style=\'display: none\'>" + path + "</div>";
    }

    public static void printTestResults(List<TestResult> testResults, String path){
        printTestResults(testResults, path, false);
    }

    public static void printTestResults(List<TestResult> testResults, String path, boolean merge){
        if ( testResults.size() == 0 ) return;
        if(merge){
            try {
                String first = FileUtils.readFileToString(new File(path));
                first = first.substring(0, first.length()-3);
                String str = first + ",\n" + printTestResults(testResults, merge);
                FileUtils.writeStringToFile(new File(path),
                        str);
            }catch (IOException e) {
                e.printStackTrace();
            }
        }

        else {
            try {
                FileUtils.writeStringToFile(new File(path),
                        printTestResults(testResults, merge));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static String printTestResults(List<TestResult> testResults, boolean merge){
        IndentPrinter ip = new IndentPrinter();
        if (!merge)
            ip.println("[");
        ip.indent();
        
        boolean first = true;
        for (TestResult testResult: testResults) {
            if(testResult == null) continue;
            int i = 0;
            
            if(!first)
                ip.print(",\n");
            else
                first = false;

            ip.println("{");
            ip.indent();
            ip.println(names[i++] + ": \"" + testResult.getModelName() + "\",");
            ip.println(names[i++] + ": \"" + modelName_hiddenPath(testResult.getModelName(), getVisulisationLink(testResult)) + "\",");
            ip.println(names[i++] + ": \"" + getVFSTag(testResult) + "\",");
            ip.println(names[i++] + ": \"" + testResult.getErrorMessages().size() + "\",");
            ip.println(names[i++] + ": \"" + testResult.getErrorMessage() + "\",");
            ip.println(names[i++] + ": \"" + testResult.getType() + "\",");
            ip.println(names[i++] + ": " + tagOf(testResult.isValid()?1:-1) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getParsed()) + ",");
            ip.println(names[i++] + ": " + tagOf(testResult.getResolve()) + ",");
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
            ip.println(names[i++] + ": " + tagOf(testResult.getAtomicComponent()));
            ip.unindent();
            ip.print("}");
        }
        ip.println();
        ip.unindent();
        ip.println("]");

        return ip.getContent();
    }

    private static String getVFSTag(TestResult testResult) {
        String zipName = testResult.getZipName();
        File file = testResult.getFilePath();
        File project = testResult.getProjectPath();
        String urlToZip;
        String zipName_;
        if(zipName == null)
            zipName_ = "models1a6a7c6e450b6d996a79c701efdd4e69.zip";
        else
            zipName_ = zipName;

        urlToZip = "https://raw.githubusercontent.com/EmbeddedMontiArc/reporting/gh-pages/" + zipName_;
        zipName_ = zipName_.substring(0, zipName_.lastIndexOf("."));
        String name = file.getAbsolutePath().substring(project.getAbsolutePath().length() - project.getName().length());
        String displayName = name;
        if(project.getAbsolutePath().contains("MontiSim"))
            displayName = "MontiSim/" + displayName;
        return ("<a target='_blank' href='onlineIDE/api/load.html?mountPoint=EmbeddedMontiArc/reporting/" + zipName_ + "&url="
                + urlToZip + "&openFile=/" + name + "'>" +
                "<img border='0' alt='" + displayName + "' src='images/favicon.ico'>" +
                "</a>").replace("\\","/");
    }

    private static String getVisulisationLink(TestResult testResult) {
        File file = testResult.getFilePath();
        File project = testResult.getProjectPath();
        String name = file.getAbsolutePath().substring(project.getAbsolutePath().length() - project.getName().length());
        String displayName = name;

        if(project.getAbsolutePath().contains("MontiSim")) {
            displayName = "MontiSim/" + displayName;
        }
        if(testResult.getSvgPath().equals(""))
            return displayName;
        else
            return "<a target='_blank' href='" +
                        testResult.getSvgPath().replace("\\","/") + ".html'>" +
                        displayName +
                    "</a>";
    }

    public static void printTestsEndWithTestResults(List<TestsEndWithTestResult> results, String path){
        printTestsEndWithTestResults(results, path, false);
    }

    public static void printTestsEndWithTestResults(List<TestsEndWithTestResult> results, String path, boolean merge){
        if ( results.size() == 0 ) return;
        if(merge){
            try {
                String first = FileUtils.readFileToString(new File(path));
                first = first.substring(0, first.length()-2);
                String str = first + ",\n" + printTestsEndWithTestResults(results, merge);
                FileUtils.writeStringToFile(new File(path),
                        str);
            }catch (IOException e) {
                e.printStackTrace();
            }
        }
        else
        try {
            FileUtils.writeStringToFile(new File(path), printTestsEndWithTestResults(results, merge));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static String printTestsEndWithTestResults(List<TestsEndWithTestResult> results, boolean merge){
        IndentPrinter ip = new IndentPrinter();
        if(!merge)
            ip.println("[");
        ip.indent();

        boolean first = true;
        for (TestsEndWithTestResult testResult: results) {
            if(testResult == null /*testResult.isValid()*/) continue;

            if(!first)
                ip.print(",\n");
            else
                first = false;

            ip.println("{");
            ip.indent();
            ip.println("\"Path\": \"" + testResult.getPath().replace("\\","/") + "\",");
            ip.println("\"NameEndsWithTest\": " + tagOf(testResult.endsWithTest()?1:-1));
            ip.unindent();
            ip.print("}");
        }
        ip.println();
        ip.unindent();
        ip.println("]");

        return ip.getContent();
    }
}
