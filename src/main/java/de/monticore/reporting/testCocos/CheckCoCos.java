package de.monticore.reporting.testCocos;

import de.monticore.reporting.testCocos.helper.CustomPrinter;
import de.monticore.reporting.testCocos.helper.SearchFiles;
import de.monticore.reporting.testCocos.helper.CheckCoCoResult;

import java.io.File;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class CheckCoCos {

    public List<CheckCoCoResult> testAllCocos(File root, String zipName, String... fileType) {
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
            for(File file: filesMap.get(projectDir)) {
                CustomPrinter.println("[" + getFormattedNumber(z, max) + "/" + max + "]" +
                        " Test CoCos of file \"" + file.getAbsolutePath());
//                    GitHubHelper ghh = new GitHubHelper();
//                    String gitHubRoot = ghh.getGitHubRoot(projectDir);
                z++;
                CheckCoCo ccT = new CheckCoCo();
                CheckCoCoResult testResult = null;

                testResult = ccT.testCoCos(file.getAbsolutePath());
                testResult.setModelFile(file);
                testResult.setProjectFile(projectDir);
//                    testResult.setPathToFile(ghh.getHTMLTagOf(projectDir, file, gitHubRoot));
                testResult.setOnlineIDE(getVFSTag(projectDir, file, zipName));
                String relativeProject = projectDir.getName();
                testResult.setProject(relativeProject);
                testResult.setRootFile(root);
                testResult.setZipName(zipName);

                testResults.add(testResult);
            }
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
