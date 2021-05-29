/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

import com.github.cliftonlabs.json_simple.JsonArray;
import com.github.cliftonlabs.json_simple.JsonException;
import com.github.cliftonlabs.json_simple.JsonObject;
import com.github.cliftonlabs.json_simple.Jsoner;
import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.reporting.Main;
import de.monticore.reporting.helper.OrderableModelInfo;
import de.monticore.reporting.order.OrderTestResults;
import de.monticore.reporting.tools.CustomPrinter;
import org.apache.commons.io.FileUtils;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class CoCoTestResultPrinterManager {
    public static void printForAllProjects(List<CheckCoCoResult> testResults, Map<String, List<CheckCoCoResult>> mainPackageModels, Main.ReportContext context) {
        File projectsDir = new File(context.getOutput() + "projects");
        projectsDir.mkdirs();
        for (String project: mainPackageModels.keySet()){
            List<CheckCoCoResult> mainPackages = mainPackageModels.get(project);
            if (mainPackages.size() == 0) continue;
            CustomPrinter.println("\n<=======Project=======> " + project);
            List<CheckCoCoResult> projectTestResults = testResults;
            String outputPostFix = "";
            boolean print = true;
            if(!project.equals(OrderTestResults.allProjects)){
                projectTestResults = testResults.stream().filter(
                        t -> t.getProject().contains(project)
                ).collect(Collectors.toList());
                outputPostFix = "projects/" + project + "/";
                File projectDir = new File(projectsDir.getAbsolutePath() + "/" + project);
                projectDir.mkdirs();
                print = true;
            }

            List<CheckCoCoResult> projectTestResultsParsingErrored = mainPackages.stream().filter(
                    t -> t.getProject().contains("Errored_Parsing")
            ).collect(Collectors.toList());
            List<CheckCoCoResult> projectTestResultsResolvingErrored = mainPackages.stream().filter(
                    t -> t.getProject().contains("Errored_Resolving")
            ).collect(Collectors.toList());
            mainPackages = mainPackages.stream().filter(
                    t -> (!t.getProject().contains("Errored_Parsing") && !t.getProject().contains("Errored_Resolving"))
            ).collect(Collectors.toList());

            String dataOut = context.getOutput() + outputPostFix + "data.json";
            String dataParsingErroredOut = context.getOutput() + outputPostFix + "dataParsingErrored.json";
            String dataResolvingErroredOut = context.getOutput() + outputPostFix + "dataResolvingErrored.json";
            String dataExtendedOut = context.getOutput() + outputPostFix + "dataExpanded.json";

            CoCoTestResultPrinter.printTestResults(mainPackages, dataOut, context.isMerge(), true, print);
            CoCoTestResultPrinter.printTestResults(projectTestResultsParsingErrored, dataParsingErroredOut, context.isMerge(), true, print);
            CoCoTestResultPrinter.printTestResults(projectTestResultsResolvingErrored, dataResolvingErroredOut, context.isMerge(), true, print);
            CoCoTestResultPrinter.printTestResults(projectTestResults, dataExtendedOut, context.isMerge(), false, print);

            TestInfoPrinter.printInfo(projectTestResults, context.getOutput() + outputPostFix + "info.json", context.getDate());
        }

        printProjectsList(context, mainPackageModels);
    }

    private static void printProjectsList(Main.ReportContext context, Map<String, List<CheckCoCoResult>> mainPackageModels) {
        File projectsList = new File(context.getOutput() + "projects/projectsList.json");
        String toPrint = getPrintString(context, mainPackageModels);
        if (context.isMerge()) {
            try {
                if (toPrint.equals("]")) return;
                String first = FileUtils.readFileToString(projectsList);
                first = first.substring(0, first.length() - 1);
                String needComma = ", ";
                if (first.equals("[")) needComma = "";
                String str = first + needComma + toPrint;
                FileUtils.writeStringToFile(projectsList,
                        str);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            try {
                FileUtils.writeStringToFile(projectsList, toPrint);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private static String getPrintString(Main.ReportContext context, Map<String, List<CheckCoCoResult>> mainPackageModels) {
        IndentPrinter ip = new IndentPrinter();
        if(!context.isMerge())
            ip.print("[");
        boolean first = true;
        for (String project: mainPackageModels.keySet()) {
            if (project.equals(OrderTestResults.allProjects)) continue;
            if (mainPackageModels.get(project).size() == 0) continue;
            if (!first)
                ip.print(", ");
            else
                first = false;
            ip.print("\"" + project + "\"");
        }
        ip.print("]");
        return ip.getContent();
    }
}
