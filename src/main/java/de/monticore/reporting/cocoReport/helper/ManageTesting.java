/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

import de.monticore.reporting.Main;
import de.monticore.reporting.cocoReport.CheckCoCos;
import de.monticore.reporting.order.OrderTestResults;
import de.monticore.reporting.tools.CustomPrinter;
import de.monticore.reporting.tools.SearchFiles;

import java.io.File;
import java.util.*;

public class ManageTesting {
    private static final String[] fileTypes = {"ema", "emam", "emadl"};
    private static final int maxToTest = 1000;

    public static void manageTesting(Main.ReportContext context) {
        List<Set<String>> projectsToTestList = partitionProjects(context);
        boolean originalIsMerge = context.isMerge();

        boolean severalSets = projectsToTestList.size() > 1 ? true : false;
        int i = 1;
        int max = projectsToTestList.size();
        boolean first = true;
        for (Set<String> projectsToTest: projectsToTestList) {
            String numberCount = severalSets ? " (" + i + "/" + max + ")" : "";
            CheckCoCos tcc = new CheckCoCos();
            CustomPrinter.println("\n<================Test CoCos================>" + numberCount + "\n");
            List<CheckCoCoResult> testResults = tcc.testAllCocos(context, projectsToTest, numberCount, fileTypes);
            Map<String, List<CheckCoCoResult>> mainPackageModels = OrderTestResults.orderTestResults(
                    new File(context.getProjectRoot()), projectsToTest, testResults, new CheckCoCoResultCreator());

            CustomPrinter.println("\n<============Write Test Results============>" + numberCount + "\n");
            CoCoTestResultPrinterManager.printForAllProjects(testResults, mainPackageModels, context);
            CustomPrinter.println("SUCCESS\n");

            if(first && severalSets) {
                first = false;
                context.setMerge(true);
            }
            i++;
        }
        context.setMerge(originalIsMerge);
    }

    private static List<Set<String>> partitionProjects(Main.ReportContext context) {
        List<Set<String>> partitions = new LinkedList<>();
        File root = new File(context.getProjectRoot());

        boolean alreadyAddedAProject = false;
        int currentTotal = 0;
        Set<String> currentSet = null;
        for (File projectDir : root.listFiles()) {
            if (projectDir.isDirectory()) {
                List<File> files = SearchFiles.searchFiles(projectDir, fileTypes);

                if (!alreadyAddedAProject || alreadyAddedAProject && currentTotal + files.size() >= maxToTest){
                    currentSet = new HashSet<>();
                    partitions.add(currentSet);
                    currentSet.add(projectDir.getName());
                    alreadyAddedAProject = true;
                    currentTotal = files.size();
                } else {
                    currentSet.add(projectDir.getName());
                    currentTotal += files.size();
                }
            }
        }

        return partitions;
    }
}
