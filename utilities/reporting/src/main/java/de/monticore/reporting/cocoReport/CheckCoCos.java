/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport;

import de.monticore.reporting.Main;
import de.monticore.reporting.tools.CustomPrinter;
import de.monticore.reporting.tools.GitLabHelper;
import de.monticore.reporting.tools.SearchFiles;
import de.monticore.reporting.cocoReport.helper.CheckCoCoResult;
import java.io.File;
import java.util.*;

public class CheckCoCos {

    public List<CheckCoCoResult> testAllCocos(Main.ReportContext context, Set<String> projectsToTest, String numberCount, String... fileType) {
        File root = new File(context.getProjectRoot());
        List<CheckCoCoResult> testResults = new LinkedList<>();

        Map<File, List<File>> filesMap = new HashMap<>();
        for (File projectDir : root.listFiles()) {
            if (projectDir.isDirectory() && projectsToTest.contains(projectDir.getName())) {
                List<File> files = SearchFiles.searchFiles(projectDir, fileType);
                filesMap.put(projectDir, files);
            }
        }
        int max = 0;
        int z   = 1;
        for (File projectDir : filesMap.keySet())
            max += filesMap.get(projectDir).size();

        for (File projectDir : filesMap.keySet()) {
            String gitLabRoot = null;
            try {
                gitLabRoot = GitLabHelper.getGitLabRoot(projectDir);
            } catch (Exception e) {
                e.printStackTrace();
            }

            for(File file: filesMap.get(projectDir)) {
                String fileName = file.getAbsolutePath().substring(context.getProjectRoot().length() + 1);
                CustomPrinter.println("[" + getFormattedNumber(z, max) + "/" + max + "]" + numberCount +
                        " Test CoCos of file " + fileName);
                z++;
                if(z == 70) {
                    int i = 9;
                }
                CheckCoCo ccT = new CheckCoCo();
                CheckCoCoResult testResult = null;

                testResult = ccT.testCoCos(file.getAbsolutePath(), context.getTimeout(), context.getCoCoTimeOut());

                testResult.setModelFile(file);
                String relativeProject = projectDir.getName();
                testResult.setProject(relativeProject);
                testResult.setRootFile(root);
                String relativeModelPath = file.getAbsolutePath().substring(
                        file.getAbsolutePath().lastIndexOf(relativeProject) + relativeProject.length() + 1)
                        .replace("\\", "/");
                testResult.setGitLabLink(gitLabRoot + relativeModelPath);

                testResults.add(testResult);
            }
        }

        return testResults;
    }

    private String getFormattedNumber(int z, int max) {
        int blanks = ((int) Math.log10(max)) - ((int) Math.log10(z));
        String blankString = "";
        for (int i = 0; i < blanks; i++)
            blankString += " ";
        return blankString + z;
    }
}
