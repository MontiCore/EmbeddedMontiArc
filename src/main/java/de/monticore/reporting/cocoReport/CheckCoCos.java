/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport;

import de.monticore.reporting.cocoReport.helper.RewriteWithoutArray;
import de.monticore.reporting.tools.CustomPrinter;
import de.monticore.reporting.tools.GitLabHelper;
import de.monticore.reporting.tools.SearchFiles;
import de.monticore.reporting.cocoReport.helper.CheckCoCoResult;
import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteWatchdog;
import org.apache.commons.exec.PumpStreamHandler;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.util.*;

public class CheckCoCos {

    public List<CheckCoCoResult> testAllCocos(File root, int timeout, int coCoTimeOut, String... fileType) {
        List<CheckCoCoResult> testResults = new LinkedList<>();

        Map<File, List<File>> filesMap = new HashMap<>();
        for (File projectDir : root.listFiles()) {
            if (projectDir.isDirectory()) {
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
                CustomPrinter.println("[" + getFormattedNumber(z, max) + "/" + max + "]" +
                        " Test CoCos of file \"" + file.getAbsolutePath());
                z++;
                if(z == 70) {
                    int i = 9;
                }
                CheckCoCo ccT = new CheckCoCo();
                CheckCoCoResult testResult = null;

                testResult = ccT.testCoCos(file.getAbsolutePath(), timeout, coCoTimeOut);

                testResult.setModelFile(file);
                String relativeProject = projectDir.getName();
                testResult.setProject(relativeProject);
                testResult.setRootFile1(root);
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
