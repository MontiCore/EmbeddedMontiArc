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

    public List<CheckCoCoResult> testAllCocos(File root, String... fileType) {
        List<CheckCoCoResult> testResults = new LinkedList<>();

        CustomPrinter.init();

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

            // Save original file
//            for(File file: filesMap.get(projectDir)){
//                String oldFilePath = file.getAbsolutePath();
//                String newFilePath = oldFilePath + "_temp";
//                file.renameTo(new File(newFilePath));
//                RewriteWithoutArray.rewrite(newFilePath, oldFilePath);
//            }

            for(File file: filesMap.get(projectDir)) {
                CustomPrinter.println("[" + getFormattedNumber(z, max) + "/" + max + "]" +
                        " Test CoCos of file \"" + file.getAbsolutePath());
                z++;
                CheckCoCo ccT = new CheckCoCo();
                CheckCoCoResult testResult = null;

                testResult = ccT.testCoCos(file.getAbsolutePath());

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

            // Reset original file
//            for(File file: filesMap.get(projectDir)){
//                File newFile = new File(file.getAbsolutePath() + "_temp");
//                if(file.exists() && newFile.exists())
//                    file.delete();
//                newFile.renameTo(file);
//            }
        }

        CustomPrinter.end();

        return testResults;
    }

    private String getVFSTag(File project, File file, String zipName) {
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
        return "<a target='_blank' href='onlineIDE/api/load.html?mountPoint=EmbeddedMontiArc/reporting/" + zipName_ + "&url="
                + urlToZip + "&openFile=/" + name + "'>" + displayName + "</a>";
    }

    private String getFormattedNumber(int z, int max) {
        int blanks = ((int) Math.log10(max)) - ((int) Math.log10(z));
        String blankString = "";
        for (int i = 0; i < blanks; i++)
            blankString += " ";
        return blankString + z;
    }
}
